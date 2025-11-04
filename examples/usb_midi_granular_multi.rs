// USB MIDI Granular Synthesizer with Multiple Sources
// MIDI channels 1-4 select different source materials
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
static GRAIN_SOURCE: AtomicU8 = AtomicU8::new(0); // Which source buffer to use

static AUDIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

// Source audio buffers (2 seconds each, 4 sources)
const NUM_SOURCES: usize = 4;
const SOURCE_AUDIO_SAMPLES: usize = 48_000 * 2; // 2 seconds stereo
const SOURCE_BUFFER_SIZE: usize = SOURCE_AUDIO_SAMPLES * 2; // stereo
const TOTAL_SOURCE_SIZE: usize = SOURCE_BUFFER_SIZE * NUM_SOURCES;

// Grain settings
const MAX_GRAINS: usize = 16;
const GRAIN_SIZE_SAMPLES: usize = 2400; // 50ms at 48kHz
const GRAIN_SIZE: usize = GRAIN_SIZE_SAMPLES * 2; // stereo

static mut SOURCE_BUFFERS: *mut u32 = core::ptr::null_mut();

#[derive(Clone, Copy)]
struct Grain {
    active: bool,
    source_id: u8,        // Which source buffer (0-3)
    position: usize,      // Position in source buffer
    playback_pos: usize,  // Current position in grain
    playback_speed: f32,  // Playback speed (pitch shift)
    amplitude: f32,       // Volume
}

impl Grain {
    fn new() -> Self {
        Self {
            active: false,
            source_id: 0,
            position: 0,
            playback_pos: 0,
            playback_speed: 1.0,
            amplitude: 0.0,
        }
    }
}

static mut GRAINS: [Grain; MAX_GRAINS] = [Grain {
    active: false,
    source_id: 0,
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

    info!("Multi-Source Granular Synth audio started");

    unwrap!(
        interface
            .start_callback(move |_input, output| {
                // Check for new grain trigger
                if TRIGGER_GRAIN.swap(false, Ordering::SeqCst) {
                    let note = GRAIN_NOTE.load(Ordering::SeqCst);
                    let source_id = GRAIN_SOURCE.load(Ordering::SeqCst);

                    // Find an inactive grain slot
                    unsafe {
                        for grain in &mut GRAINS {
                            if !grain.active {
                                // Position in selected source buffer
                                let position = ((note as usize) * 1000) % (SOURCE_BUFFER_SIZE - GRAIN_SIZE);

                                // Pitch shift based on note
                                let pitch_shift = 2.0_f32.powf((note as f32 - 60.0) / 12.0);

                                grain.active = true;
                                grain.source_id = source_id;
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
                        if !SOURCE_BUFFERS.is_null() {
                            for grain in &mut GRAINS {
                                if grain.active {
                                    active_count += 1;

                                    // Calculate envelope (triangle)
                                    let progress = grain.playback_pos as f32 / GRAIN_SIZE_SAMPLES as f32;
                                    let envelope = if progress < 0.5 {
                                        progress * 2.0
                                    } else {
                                        2.0 - (progress * 2.0)
                                    };

                                    // Calculate offset into correct source buffer
                                    let source_offset = (grain.source_id as usize) * SOURCE_BUFFER_SIZE;
                                    let read_pos = source_offset + grain.position + (grain.playback_pos * 2);

                                    if read_pos + 1 < TOTAL_SOURCE_SIZE {
                                        let source = core::slice::from_raw_parts(
                                            SOURCE_BUFFERS,
                                            TOTAL_SOURCE_SIZE
                                        );

                                        let sample_l = u24_to_f32(source[read_pos]);
                                        let sample_r = u24_to_f32(source[read_pos + 1]);

                                        mix_l += sample_l * envelope * grain.amplitude;
                                        mix_r += sample_r * envelope * grain.amplitude;
                                    }

                                    grain.playback_pos += 1;

                                    if grain.playback_pos >= GRAIN_SIZE_SAMPLES {
                                        grain.active = false;
                                    }
                                }
                            }
                        }
                    }

                    // Normalize
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

                    // Extract MIDI channel (0-15 from status byte)
                    let channel = status & 0x0F;

                    // Note On triggers a grain
                    if (status & 0xF0) == 0x90 && velocity > 0 {
                        // Map MIDI channels 1-4 to sources 0-3
                        let source_id = if channel < 4 { channel } else { 0 };

                        GRAIN_NOTE.store(note, Ordering::SeqCst);
                        GRAIN_SOURCE.store(source_id, Ordering::SeqCst);
                        TRIGGER_GRAIN.store(true, Ordering::SeqCst);

                        info!("Grain: ch {} (src {}) note {}", channel + 1, source_id, note);
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
    info!("USB MIDI Multi-Source Granular Synthesizer");
    info!("SDRAM Size: {} MB", SDRAM_SIZE / (1024 * 1024));
    info!("Sources: {}, {} samples each ({} sec)",
          NUM_SOURCES, SOURCE_AUDIO_SAMPLES, SOURCE_AUDIO_SAMPLES / 48000);
    info!("Total source memory: {} KB", (TOTAL_SOURCE_SIZE * 4) / 1024);
    info!("Max grains: {}, Grain size: {}ms", MAX_GRAINS, GRAIN_SIZE_SAMPLES * 1000 / 48000);
    info!("");
    info!("MIDI Channel 1 = Sine Sweep");
    info!("MIDI Channel 2 = Square Wave");
    info!("MIDI Channel 3 = Sawtooth Wave");
    info!("MIDI Channel 4 = Noise Burst");

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

    info!("Generating source audio...");

    // Source 0: Sine wave sweep (100Hz → 1000Hz)
    info!("  Source 0: Sine sweep");
    let mut phase = 0.0f32;
    for i in 0..SOURCE_AUDIO_SAMPLES {
        let progress = i as f32 / SOURCE_AUDIO_SAMPLES as f32;
        let freq = 100.0 + (progress * 900.0);
        phase += freq * 2.0 * core::f32::consts::PI / 48000.0;
        if phase >= 2.0 * core::f32::consts::PI {
            phase -= 2.0 * core::f32::consts::PI;
        }
        let sample = f32_to_u24(phase.sin() * 0.5);
        ram_slice[i * 2] = sample;
        ram_slice[i * 2 + 1] = sample;
    }

    // Source 1: Square wave (200Hz → 800Hz)
    info!("  Source 1: Square wave");
    phase = 0.0;
    let offset = SOURCE_BUFFER_SIZE;
    for i in 0..SOURCE_AUDIO_SAMPLES {
        let progress = i as f32 / SOURCE_AUDIO_SAMPLES as f32;
        let freq = 200.0 + (progress * 600.0);
        phase += freq * 2.0 * core::f32::consts::PI / 48000.0;
        if phase >= 2.0 * core::f32::consts::PI {
            phase -= 2.0 * core::f32::consts::PI;
        }
        let square = if phase < core::f32::consts::PI { 0.5 } else { -0.5 };
        let sample = f32_to_u24(square);
        ram_slice[offset + i * 2] = sample;
        ram_slice[offset + i * 2 + 1] = sample;
    }

    // Source 2: Sawtooth wave (150Hz → 1200Hz)
    info!("  Source 2: Sawtooth wave");
    phase = 0.0;
    let offset = SOURCE_BUFFER_SIZE * 2;
    for i in 0..SOURCE_AUDIO_SAMPLES {
        let progress = i as f32 / SOURCE_AUDIO_SAMPLES as f32;
        let freq = 150.0 + (progress * 1050.0);
        phase += freq * 2.0 * core::f32::consts::PI / 48000.0;
        if phase >= 2.0 * core::f32::consts::PI {
            phase -= 2.0 * core::f32::consts::PI;
        }
        let saw = (phase / core::f32::consts::PI) - 1.0;
        let sample = f32_to_u24(saw * 0.5);
        ram_slice[offset + i * 2] = sample;
        ram_slice[offset + i * 2 + 1] = sample;
    }

    // Source 3: Filtered noise (percussive)
    info!("  Source 3: Noise burst");
    let offset = SOURCE_BUFFER_SIZE * 3;
    let mut noise_state: u32 = 12345;
    for i in 0..SOURCE_AUDIO_SAMPLES {
        // Simple LFSR noise generator
        noise_state = noise_state.wrapping_mul(1664525).wrapping_add(1013904223);
        let noise_val = ((noise_state >> 16) as i32 - 32768) as f32 / 32768.0;

        // Envelope: decay over time
        let decay = 1.0 - (i as f32 / SOURCE_AUDIO_SAMPLES as f32);
        let sample = f32_to_u24(noise_val * 0.3 * decay);

        ram_slice[offset + i * 2] = sample;
        ram_slice[offset + i * 2 + 1] = sample;
    }

    unsafe {
        SOURCE_BUFFERS = ram_slice.as_mut_ptr();
    }

    info!("All sources generated - ready!");

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
    usb_config.product = Some("Multi Granular");
    usb_config.serial_number = Some("MGRAN01");

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
            info!("MIDI Connected - trigger grains on channels 1-4!");
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
