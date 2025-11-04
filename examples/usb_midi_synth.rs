// USB MIDI Synthesizer - Proper 4-byte USB MIDI packet parsing
// This is the correct way to parse USB MIDI with embassy-usb
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU8, AtomicBool, Ordering};

use daisy_embassy::hal::{bind_interrupts, peripherals, usb, interrupt};
use daisy_embassy::new_daisy_board;
use defmt::{info, unwrap};
use embassy_executor::{Spawner, InterruptExecutor};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::usb::{Config, Driver, Instance};
use embassy_usb::class::midi::MidiClass;
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use micromath::F32Ext;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(pub struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

static CURRENT_NOTE: AtomicU8 = AtomicU8::new(60);
static NOTE_ACTIVE: AtomicBool = AtomicBool::new(true);

static AUDIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SAI1() {
    AUDIO_EXECUTOR.on_interrupt()
}

fn midi_to_freq(note: u8) -> f32 {
    440.0 * 2.0_f32.powf((note as f32 - 69.0) / 12.0)
}

#[embassy_executor::task]
async fn audio_task(
    interface: daisy_embassy::audio::Interface<'static, daisy_embassy::audio::Idle>,
) {
    let mut interface = unwrap!(interface.start_interface().await);

    let mut phase = 0.0f32;
    let mut last_note = 60u8;

    info!("USB MIDI Synth audio started");

    unwrap!(
        interface
            .start_callback(move |_input, output| {
                let note = CURRENT_NOTE.load(Ordering::SeqCst);
                let active = NOTE_ACTIVE.load(Ordering::SeqCst);

                if note != last_note {
                    last_note = note;
                    phase = 0.0;
                }

                if active {
                    let freq = midi_to_freq(last_note);

                    for chunk in output.chunks_mut(2) {
                        phase += freq * 2.0 * core::f32::consts::PI / 48000.0;
                        if phase >= 2.0 * core::f32::consts::PI {
                            phase -= 2.0 * core::f32::consts::PI;
                        }

                        let sample = f32_to_u24(phase.sin() * 0.5);
                        chunk[0] = sample;
                        chunk[1] = sample;
                    }
                } else {
                    for chunk in output.chunks_mut(2) {
                        chunk[0] = 0;
                        chunk[1] = 0;
                    }
                    phase = 0.0;
                }
            })
            .await
    );
}

async fn midi_handler<'d, T: Instance + 'd>(
    class: &mut MidiClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];

    loop {
        let n = class.read_packet(&mut buf).await?;

        if n > 0 {
            let data = &buf[..n];

            // CRITICAL: USB MIDI packets are 4 bytes: [header, status, data1, data2]
            // Embassy-usb MidiClass does NOT strip the header - parse as 4-byte chunks!
            for packet in data.chunks(4) {
                if packet.len() >= 4 {
                    let _header = packet[0];  // Cable number + Code Index Number
                    let status = packet[1];    // MIDI status byte
                    let note = packet[2];      // Note number (0-127)
                    let velocity = packet[3];  // Velocity (0-127)

                    // Note On (0x90-0x9F) with velocity > 0
                    if (status & 0xF0) == 0x90 && velocity > 0 {
                        CURRENT_NOTE.store(note, Ordering::SeqCst);
                        NOTE_ACTIVE.store(true, Ordering::SeqCst);
                        info!("Note On: {} ({}Hz)", note, midi_to_freq(note) as u32);
                    }
                    // Note Off (0x80-0x8F) or Note On with velocity 0
                    else if (status & 0xF0) == 0x80 || ((status & 0xF0) == 0x90 && velocity == 0) {
                        if note == CURRENT_NOTE.load(Ordering::SeqCst) {
                            NOTE_ACTIVE.store(false, Ordering::SeqCst);
                            info!("Note Off: {}", note);
                        }
                    }
                }
            }
        }
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => {
                info!("Buffer overflow");
                Disconnected {}
            }
            EndpointError::Disabled => Disconnected {},
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("USB MIDI Synthesizer");

    let p = embassy_stm32::init(daisy_embassy::default_rcc());
    let board = new_daisy_board!(p);

    // Start audio in high-priority interrupt executor
    let interface = board.audio_peripherals.prepare_interface(Default::default()).await;
    interrupt::SAI1.set_priority(Priority::P6);
    let audio_spawner = AUDIO_EXECUTOR.start(interrupt::SAI1);
    unwrap!(audio_spawner.spawn(audio_task(interface)));

    // USB MIDI setup
    let mut config = Config::default();
    config.vbus_detection = false;

    static EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
    let ep_out_buffer = EP_OUT_BUFFER.init([0; 256]);

    let driver = Driver::new_fs(
        board.usb_peripherals.usb_otg_fs,
        Irqs,
        board.usb_peripherals.pins.DP,
        board.usb_peripherals.pins.DN,
        ep_out_buffer,
        config,
    );

    let mut usb_config = embassy_usb::Config::new(0x1234, 0x5678);
    usb_config.manufacturer = Some("Daisy-Embassy");
    usb_config.product = Some("USB MIDI Synth");
    usb_config.serial_number = Some("SYNTH01");

    usb_config.device_class = 0xEF;
    usb_config.device_sub_class = 0x02;
    usb_config.device_protocol = 0x01;
    usb_config.composite_with_iads = true;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );

    let mut class = MidiClass::new(&mut builder, 1, 1, 64);
    let mut usb = builder.build();

    let usb_fut = usb.run();
    let midi_fut = async {
        loop {
            class.wait_connection().await;
            info!("MIDI Connected");
            let _ = midi_handler(&mut class).await;
            info!("MIDI Disconnected");
        }
    };

    embassy_futures::join::join(usb_fut, midi_fut).await;
}

#[inline(always)]
fn f32_to_u24(x: f32) -> u32 {
    let x = x * 8_388_607.0;
    let x = x.clamp(-8_388_608.0, 8_388_607.0);
    (x as i32) as u32
}
