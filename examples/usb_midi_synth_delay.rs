// USB MIDI Synthesizer with SDRAM Delay Effect
// Demonstrates proper 4-byte USB MIDI parsing + 64MB SDRAM usage
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU8, AtomicBool, Ordering};

use daisy_embassy::hal::{bind_interrupts, peripherals, usb, interrupt};
use daisy_embassy::{new_daisy_board, sdram::SDRAM_SIZE};
use defmt::{info, unwrap};
use embassy_executor::{Spawner, InterruptExecutor};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::usb::{Config, Driver, Instance};
use embassy_time::Delay;
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

// SDRAM delay buffer
// 48kHz stereo = 96k samples/sec
// 2 seconds delay = 192k samples = 768KB
const DELAY_TIME_SAMPLES: usize = 96_000; // 1 second stereo
const DELAY_BUFFER_SIZE: usize = DELAY_TIME_SAMPLES * 2; // stereo
static mut DELAY_BUFFER: *mut u32 = core::ptr::null_mut();
static mut DELAY_POS: usize = 0;

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

    info!("USB MIDI Synth with SDRAM Delay started");

    unwrap!(
        interface
            .start_callback(move |_input, output| {
                let note = CURRENT_NOTE.load(Ordering::SeqCst);
                let active = NOTE_ACTIVE.load(Ordering::SeqCst);

                if note != last_note {
                    last_note = note;
                    phase = 0.0;
                }

                let freq = midi_to_freq(last_note);

                // Process audio with delay
                for chunk in output.chunks_mut(2) {
                    // Generate synth tone
                    let synth_sample = if active {
                        phase += freq * 2.0 * core::f32::consts::PI / 48000.0;
                        if phase >= 2.0 * core::f32::consts::PI {
                            phase -= 2.0 * core::f32::consts::PI;
                        }
                        phase.sin() * 0.3  // Dry signal at 30%
                    } else {
                        phase = 0.0;
                        0.0
                    };

                    // Read from delay buffer (50% wet)
                    let (delayed_l, delayed_r) = unsafe {
                        if !DELAY_BUFFER.is_null() {
                            let buffer = core::slice::from_raw_parts_mut(
                                DELAY_BUFFER,
                                DELAY_BUFFER_SIZE
                            );

                            let delay_l = u24_to_f32(buffer[DELAY_POS]) * 0.5;
                            let delay_r = u24_to_f32(buffer[DELAY_POS + 1]) * 0.5;

                            (delay_l, delay_r)
                        } else {
                            (0.0, 0.0)
                        }
                    };

                    // Mix dry + wet
                    let mixed_l = synth_sample + delayed_l;
                    let mixed_r = synth_sample + delayed_r;

                    // Convert to u24
                    let out_l = f32_to_u24(mixed_l);
                    let out_r = f32_to_u24(mixed_r);

                    // Write to output
                    chunk[0] = out_l;
                    chunk[1] = out_r;

                    // Write to delay buffer for next iteration
                    unsafe {
                        if !DELAY_BUFFER.is_null() {
                            let buffer = core::slice::from_raw_parts_mut(
                                DELAY_BUFFER,
                                DELAY_BUFFER_SIZE
                            );

                            buffer[DELAY_POS] = out_l;
                            buffer[DELAY_POS + 1] = out_r;

                            DELAY_POS = (DELAY_POS + 2) % DELAY_BUFFER_SIZE;
                        }
                    }
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

            // CRITICAL: USB MIDI packets are 4 bytes!
            for packet in data.chunks(4) {
                if packet.len() >= 4 {
                    let _header = packet[0];
                    let status = packet[1];
                    let note = packet[2];
                    let velocity = packet[3];

                    if (status & 0xF0) == 0x90 && velocity > 0 {
                        CURRENT_NOTE.store(note, Ordering::SeqCst);
                        NOTE_ACTIVE.store(true, Ordering::SeqCst);
                        info!("Note On: {} ({}Hz)", note, midi_to_freq(note) as u32);
                    }
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
    info!("USB MIDI Synth with SDRAM Delay");
    info!("SDRAM Size: {} MB", SDRAM_SIZE / (1024 * 1024));
    info!("Delay buffer: {} samples ({} sec stereo)",
          DELAY_TIME_SAMPLES,
          DELAY_TIME_SAMPLES / 48000);

    let p = embassy_stm32::init(daisy_embassy::default_rcc());
    let board = new_daisy_board!(p);

    // Initialize SDRAM
    let mut core = cortex_m::Peripherals::take().unwrap();
    let mut sdram = board.sdram.build(&mut core.MPU, &mut core.SCB);
    let mut delay = Delay;

    let ram_slice = unsafe {
        let ram_ptr: *mut u32 = sdram.init(&mut delay) as *mut _;
        core::slice::from_raw_parts_mut(ram_ptr, SDRAM_SIZE / core::mem::size_of::<u32>())
    };

    // Initialize delay buffer to zero
    for i in 0..DELAY_BUFFER_SIZE.min(ram_slice.len()) {
        ram_slice[i] = 0;
    }

    // Set global pointer to SDRAM
    unsafe {
        DELAY_BUFFER = ram_slice.as_mut_ptr();
        DELAY_POS = 0;
    }

    info!("SDRAM initialized - delay buffer ready");

    // Start audio
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
    usb_config.product = Some("MIDI Delay");
    usb_config.serial_number = Some("DELAY01");

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

#[inline(always)]
fn u24_to_f32(x: u32) -> f32 {
    let x = x as i32;
    (x as f32) / 8_388_607.0
}
