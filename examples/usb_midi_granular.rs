// USB MIDI Granular Synthesizer with SDRAM
// Store source audio in SDRAM and trigger grains via MIDI
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

static TRIGGER_GRAIN: AtomicBool = AtomicBool::new(false);
static GRAIN_NOTE: AtomicU8 = AtomicU8::new(60);

static AUDIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

// Source audio buffer (2 seconds stereo at 48kHz)
const SOURCE_AUDIO_SAMPLES: usize = 48_000 * 2; // 2 seconds stereo
const SOURCE_BUFFER_SIZE: usize = SOURCE_AUDIO_SAMPLES * 2; // stereo

// Grain settings
const MAX_GRAINS: usize = 16; // Play up to 16 grains simultaneously
const GRAIN_SIZE_SAMPLES: usize = 2400; // 50ms grain at 48kHz
const GRAIN_SIZE: usize = GRAIN_SIZE_SAMPLES * 2; // stereo

static mut SOURCE_BUFFER: *mut u32 = core::ptr::null_mut();

// Grain state
#[derive(Clone, Copy)]
struct Grain {
    active: bool,
    position: usize,      // Position in source buffer
    playback_pos: usize,  // Current position in grain
    playback_speed: f32,  // Playback speed (pitch shift)
    amplitude: f32,       // Volume
}

impl Grain {
    fn new() -> Self {
        Self {
            active: false,
            position: 0,
            playback_pos: 0,
            playback_speed: 1.0,
            amplitude: 0.0,
        }
    }
}

static mut GRAINS: [Grain; MAX_GRAINS] = [Grain {
    active: false,
    position: 0,
    playback_pos: 0,
    playback_speed: 1.0,
    amplitude: 0.0,
}; MAX_GRAINS];

#[interrupt]
unsafe fn SAI1() {
    AUDIO_EXECUTOR.on_interrupt()
}

#[embassy_executor::task]
async fn audio_task(
    interface: daisy_embassy::audio::Interface<'static, daisy_embassy::audio::Idle>,
) {
    let mut interface = unwrap!(interface.start_interface().await);

    info!("Granular Synth audio started");

    unwrap!(
        interface
            .start_callback(move |_input, output| {
                // Check for new grain trigger
                if TRIGGER_GRAIN.swap(false, Ordering::SeqCst) {
                    let note = GRAIN_NOTE.load(Ordering::SeqCst);

                    // Find an inactive grain slot
                    unsafe {
                        for grain in &mut GRAINS {
                            if !grain.active {
                                // Trigger new grain
                                // Note determines position in source buffer
                                let position = ((note as usize) * 1000) % (SOURCE_BUFFER_SIZE - GRAIN_SIZE);

                                // Pitch shift based on note distance from middle C
                                let pitch_shift = 2.0_f32.powf((note as f32 - 60.0) / 12.0);

                                grain.active = true;
                                grain.position = position;
                                grain.playback_pos = 0;
                                grain.playback_speed = pitch_shift;
                                grain.amplitude = 0.5;
                                break;
                            }
                        }
                    }
                }

                // Process all active grains
                for chunk in output.chunks_mut(2) {
                    let mut mix_l = 0.0f32;
                    let mut mix_r = 0.0f32;
                    let mut active_count = 0;

                    unsafe {
                        if !SOURCE_BUFFER.is_null() {
                            let source = core::slice::from_raw_parts(
                                SOURCE_BUFFER,
                                SOURCE_BUFFER_SIZE
                            );

                            for grain in &mut GRAINS {
                                if grain.active {
                                    active_count += 1;

                                    // Calculate envelope (simple triangle)
                                    let progress = grain.playback_pos as f32 / GRAIN_SIZE_SAMPLES as f32;
                                    let envelope = if progress < 0.5 {
                                        progress * 2.0 // Attack
                                    } else {
                                        2.0 - (progress * 2.0) // Decay
                                    };

                                    // Read from source buffer with interpolation
                                    let read_pos = grain.position + (grain.playback_pos * 2);

                                    if read_pos + 1 < SOURCE_BUFFER_SIZE {
                                        let sample_l = u24_to_f32(source[read_pos]);
                                        let sample_r = u24_to_f32(source[read_pos + 1]);

                                        mix_l += sample_l * envelope * grain.amplitude;
                                        mix_r += sample_r * envelope * grain.amplitude;
                                    }

                                    // Advance playback position
                                    grain.playback_pos += 1;

                                    // Deactivate when grain is done
                                    if grain.playback_pos >= GRAIN_SIZE_SAMPLES {
                                        grain.active = false;
                                    }
                                }
                            }
                        }
                    }

                    // Normalize if multiple grains playing
                    let gain = if active_count > 0 {
                        1.0 / (active_count as f32).sqrt()
                    } else {
                        1.0
                    };

                    chunk[0] = f32_to_u24(mix_l * gain);
                    chunk[1] = f32_to_u24(mix_r * gain);
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

            // Parse as 4-byte USB MIDI packets
            for packet in data.chunks(4) {
                if packet.len() >= 4 {
                    let _header = packet[0];
                    let status = packet[1];
                    let note = packet[2];
                    let velocity = packet[3];

                    // Note On triggers a grain
                    if (status & 0xF0) == 0x90 && velocity > 0 {
                        GRAIN_NOTE.store(note, Ordering::SeqCst);
                        TRIGGER_GRAIN.store(true, Ordering::SeqCst);
                        info!("Grain triggered: note {}", note);
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
    info!("USB MIDI Granular Synthesizer");
    info!("SDRAM Size: {} MB", SDRAM_SIZE / (1024 * 1024));
    info!("Source buffer: {} samples ({} sec stereo)",
          SOURCE_AUDIO_SAMPLES,
          SOURCE_AUDIO_SAMPLES / 48000);
    info!("Max grains: {}", MAX_GRAINS);
    info!("Grain size: {} samples ({}ms)", GRAIN_SIZE_SAMPLES, GRAIN_SIZE_SAMPLES * 1000 / 48000);

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

    // Generate source audio (sine wave sweep for demo)
    info!("Generating source audio...");
    let mut phase = 0.0f32;
    for i in 0..SOURCE_AUDIO_SAMPLES {
        // Sweep from 100Hz to 1000Hz over the buffer
        let progress = i as f32 / SOURCE_AUDIO_SAMPLES as f32;
        let freq = 100.0 + (progress * 900.0);

        phase += freq * 2.0 * core::f32::consts::PI / 48000.0;
        if phase >= 2.0 * core::f32::consts::PI {
            phase -= 2.0 * core::f32::consts::PI;
        }

        let sample = phase.sin() * 0.5;
        let sample_u24 = f32_to_u24(sample);

        // Stereo
        ram_slice[i * 2] = sample_u24;
        ram_slice[i * 2 + 1] = sample_u24;
    }

    unsafe {
        SOURCE_BUFFER = ram_slice.as_mut_ptr();
    }

    info!("Source audio generated - ready!");

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
    usb_config.product = Some("Granular");
    usb_config.serial_number = Some("GRAIN01");

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
            info!("MIDI Connected - trigger grains with notes!");
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
