// USB MIDI Granular Synthesizer with Stable SDRAM Reverb
// Optimized for stability with many simultaneous notes
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
static GRAIN_SOURCE: AtomicU8 = AtomicU8::new(0);

static AUDIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

// Source buffers
const NUM_SOURCES: usize = 4;
const SOURCE_AUDIO_SAMPLES: usize = 48_000 * 2;
const SOURCE_BUFFER_SIZE: usize = SOURCE_AUDIO_SAMPLES * 2;
const TOTAL_SOURCE_SIZE: usize = SOURCE_BUFFER_SIZE * NUM_SOURCES;

// Simplified reverb - fewer taps for stability
const NUM_REVERB_TAPS: usize = 4; // Reduced from 8
const REVERB_DELAYS: [usize; NUM_REVERB_TAPS] = [
    3137,  // ~65ms
    5381,  // ~112ms
    7963,  // ~166ms
    10271, // ~214ms
];
const MAX_REVERB_DELAY: usize = 10271 * 2;
const REVERB_BUFFER_SIZE: usize = MAX_REVERB_DELAY * NUM_REVERB_TAPS;

// Grain settings
const MAX_GRAINS: usize = 12; // Reduced from 16 for stability
const GRAIN_SIZE_SAMPLES: usize = 4800; // 100ms
const GRAIN_SIZE: usize = GRAIN_SIZE_SAMPLES * 2;

static mut SDRAM_PTR: *mut u32 = core::ptr::null_mut();
static mut SDRAM_LEN: usize = 0;

#[derive(Clone, Copy)]
struct Grain {
    active: bool,
    source_id: u8,
    position: usize,
    playback_pos: usize,
    amplitude: f32,
}

impl Grain {
    fn new() -> Self {
        Self {
            active: false,
            source_id: 0,
            position: 0,
            playback_pos: 0,
            amplitude: 0.0,
        }
    }
}

static mut GRAINS: [Grain; MAX_GRAINS] = [Grain {
    active: false,
    source_id: 0,
    position: 0,
    playback_pos: 0,
    amplitude: 0.0,
}; MAX_GRAINS];

static mut REVERB_POSITIONS: [usize; NUM_REVERB_TAPS] = [0; NUM_REVERB_TAPS];

#[interrupt]
unsafe fn SAI1() {
    AUDIO_EXECUTOR.on_interrupt()
}

#[inline(always)]
fn get_sdram_sample(index: usize) -> f32 {
    unsafe {
        if !SDRAM_PTR.is_null() && index < SDRAM_LEN {
            let sdram = core::slice::from_raw_parts(SDRAM_PTR, SDRAM_LEN);
            u24_to_f32(sdram[index])
        } else {
            0.0
        }
    }
}

#[inline(always)]
fn set_sdram_sample(index: usize, value: f32) {
    unsafe {
        if !SDRAM_PTR.is_null() && index < SDRAM_LEN {
            let sdram = core::slice::from_raw_parts_mut(SDRAM_PTR, SDRAM_LEN);
            sdram[index] = f32_to_u24(value);
        }
    }
}

#[embassy_executor::task]
async fn audio_task(
    interface: daisy_embassy::audio::Interface<'static, daisy_embassy::audio::Idle>,
) {
    let mut interface = unwrap!(interface.start_interface().await);

    info!("Stable Granular + Reverb started");

    let reverb_offset = TOTAL_SOURCE_SIZE;

    unwrap!(
        interface
            .start_callback(move |_input, output| {
                // Trigger grains
                if TRIGGER_GRAIN.swap(false, Ordering::SeqCst) {
                    let note = GRAIN_NOTE.load(Ordering::SeqCst);
                    let source_id = GRAIN_SOURCE.load(Ordering::SeqCst);

                    unsafe {
                        // Find inactive grain or replace oldest
                        let mut slot_idx = 0;
                        let mut found_inactive = false;

                        for (i, grain) in GRAINS.iter().enumerate() {
                            if !grain.active {
                                slot_idx = i;
                                found_inactive = true;
                                break;
                            }
                        }

                        // If no inactive slots, replace the most advanced grain
                        if !found_inactive {
                            let mut max_pos = 0;
                            for (i, grain) in GRAINS.iter().enumerate() {
                                if grain.playback_pos > max_pos {
                                    max_pos = grain.playback_pos;
                                    slot_idx = i;
                                }
                            }
                        }

                        let grain = &mut GRAINS[slot_idx];
                        let position = ((note as usize) * 800) % (SOURCE_BUFFER_SIZE - GRAIN_SIZE);

                        grain.active = true;
                        grain.source_id = source_id;
                        grain.position = position;
                        grain.playback_pos = 0;
                        grain.amplitude = 0.5;
                    }
                }

                // Process audio
                for chunk in output.chunks_mut(2) {
                    let mut dry_l = 0.0f32;
                    let mut dry_r = 0.0f32;
                    let mut grain_count = 0;

                    // Generate grains
                    unsafe {
                        for grain in &mut GRAINS {
                            if grain.active {
                                grain_count += 1;

                                // ASR envelope
                                let progress = grain.playback_pos as f32 / GRAIN_SIZE_SAMPLES as f32;
                                let envelope = if progress < 0.3 {
                                    progress / 0.3
                                } else if progress < 0.7 {
                                    1.0
                                } else {
                                    (1.0 - progress) / 0.3
                                };

                                let source_offset = (grain.source_id as usize) * SOURCE_BUFFER_SIZE;
                                let read_l = source_offset + grain.position + (grain.playback_pos * 2);
                                let read_r = read_l + 1;

                                if read_r < TOTAL_SOURCE_SIZE {
                                    let sample_l = get_sdram_sample(read_l);
                                    let sample_r = get_sdram_sample(read_r);

                                    dry_l += sample_l * envelope * grain.amplitude;
                                    dry_r += sample_r * envelope * grain.amplitude;
                                }

                                grain.playback_pos += 1;
                                if grain.playback_pos >= GRAIN_SIZE_SAMPLES {
                                    grain.active = false;
                                }
                            }
                        }
                    }

                    // Normalize
                    if grain_count > 0 {
                        let gain = 0.6 / (grain_count as f32).sqrt();
                        dry_l *= gain;
                        dry_r *= gain;
                    }

                    // Simplified reverb processing
                    let mut reverb_l = 0.0f32;
                    let mut reverb_r = 0.0f32;

                    unsafe {
                        for tap in 0..NUM_REVERB_TAPS {
                            let delay_samples = REVERB_DELAYS[tap];
                            let tap_offset = reverb_offset + (tap * MAX_REVERB_DELAY);
                            let pos = REVERB_POSITIONS[tap];

                            let read_l = tap_offset + pos;
                            let read_r = tap_offset + pos + 1;

                            // Bounds check
                            if read_r < reverb_offset + REVERB_BUFFER_SIZE {
                                let delayed_l = get_sdram_sample(read_l);
                                let delayed_r = get_sdram_sample(read_r);

                                // Decay per tap
                                let decay = 0.4 - (tap as f32 * 0.08);
                                reverb_l += delayed_l * decay;
                                reverb_r += delayed_r * decay;

                                // Feedback
                                let feedback = 0.45;
                                let new_l = dry_l * 0.3 + delayed_l * feedback;
                                let new_r = dry_r * 0.3 + delayed_r * feedback;

                                set_sdram_sample(read_l, new_l);
                                set_sdram_sample(read_r, new_r);

                                // Advance
                                let next_pos = pos + 2;
                                REVERB_POSITIONS[tap] = if next_pos >= delay_samples * 2 {
                                    0
                                } else {
                                    next_pos
                                };
                            }
                        }
                    }

                    // Mix 60% dry, 40% wet
                    let out_l = dry_l * 0.6 + reverb_l * 0.4;
                    let out_r = dry_r * 0.6 + reverb_r * 0.4;

                    chunk[0] = f32_to_u24(out_l);
                    chunk[1] = f32_to_u24(out_r);
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

            for packet in data.chunks(4) {
                if packet.len() >= 4 {
                    let _header = packet[0];
                    let status = packet[1];
                    let note = packet[2];
                    let velocity = packet[3];

                    let channel = status & 0x0F;

                    if (status & 0xF0) == 0x90 && velocity > 0 {
                        let source_id = if channel < 4 { channel } else { 0 };

                        GRAIN_NOTE.store(note, Ordering::SeqCst);
                        GRAIN_SOURCE.store(source_id, Ordering::SeqCst);
                        TRIGGER_GRAIN.store(true, Ordering::SeqCst);
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
            EndpointError::BufferOverflow => Disconnected {},
            EndpointError::Disabled => Disconnected {},
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Stable Granular Synth with Reverb");
    info!("Max grains: {} (reduced for stability)", MAX_GRAINS);
    info!("Reverb taps: {} (optimized)", NUM_REVERB_TAPS);

    let p = embassy_stm32::init(daisy_embassy::default_rcc());
    let board = new_daisy_board!(p);

    let mut core = cortex_m::Peripherals::take().unwrap();
    let mut sdram = board.sdram.build(&mut core.MPU, &mut core.SCB);
    let mut delay = Delay;

    let ram_slice = unsafe {
        let ram_ptr: *mut u32 = sdram.init(&mut delay) as *mut _;
        core::slice::from_raw_parts_mut(ram_ptr, SDRAM_SIZE / core::mem::size_of::<u32>())
    };

    info!("Generating sources...");

    // Generate sources (same as before)
    let mut phase = 0.0f32;
    for i in 0..SOURCE_AUDIO_SAMPLES {
        let progress = i as f32 / SOURCE_AUDIO_SAMPLES as f32;
        let freq = 100.0 + (progress * 900.0);
        phase += freq * 2.0 * core::f32::consts::PI / 48000.0;
        if phase >= 2.0 * core::f32::consts::PI { phase -= 2.0 * core::f32::consts::PI; }
        ram_slice[i * 2] = f32_to_u24(phase.sin() * 0.5);
        ram_slice[i * 2 + 1] = ram_slice[i * 2];
    }

    phase = 0.0;
    for i in 0..SOURCE_AUDIO_SAMPLES {
        let progress = i as f32 / SOURCE_AUDIO_SAMPLES as f32;
        let freq = 200.0 + (progress * 600.0);
        phase += freq * 2.0 * core::f32::consts::PI / 48000.0;
        if phase >= 2.0 * core::f32::consts::PI { phase -= 2.0 * core::f32::consts::PI; }
        let square = if phase < core::f32::consts::PI { 0.5 } else { -0.5 };
        let offset = SOURCE_BUFFER_SIZE;
        ram_slice[offset + i * 2] = f32_to_u24(square);
        ram_slice[offset + i * 2 + 1] = ram_slice[offset + i * 2];
    }

    phase = 0.0;
    for i in 0..SOURCE_AUDIO_SAMPLES {
        let progress = i as f32 / SOURCE_AUDIO_SAMPLES as f32;
        let freq = 150.0 + (progress * 1050.0);
        phase += freq * 2.0 * core::f32::consts::PI / 48000.0;
        if phase >= 2.0 * core::f32::consts::PI { phase -= 2.0 * core::f32::consts::PI; }
        let saw = (phase / core::f32::consts::PI) - 1.0;
        let offset = SOURCE_BUFFER_SIZE * 2;
        ram_slice[offset + i * 2] = f32_to_u24(saw * 0.5);
        ram_slice[offset + i * 2 + 1] = ram_slice[offset + i * 2];
    }

    let mut noise_state: u32 = 12345;
    for i in 0..SOURCE_AUDIO_SAMPLES {
        noise_state = noise_state.wrapping_mul(1664525).wrapping_add(1013904223);
        let noise = ((noise_state >> 16) as i32 - 32768) as f32 / 32768.0;
        let decay = 1.0 - (i as f32 / SOURCE_AUDIO_SAMPLES as f32);
        let offset = SOURCE_BUFFER_SIZE * 3;
        ram_slice[offset + i * 2] = f32_to_u24(noise * 0.3 * decay);
        ram_slice[offset + i * 2 + 1] = ram_slice[offset + i * 2];
    }

    // Clear reverb buffers
    for i in TOTAL_SOURCE_SIZE..(TOTAL_SOURCE_SIZE + REVERB_BUFFER_SIZE).min(ram_slice.len()) {
        ram_slice[i] = 0;
    }

    unsafe {
        SDRAM_PTR = ram_slice.as_mut_ptr();
        SDRAM_LEN = (TOTAL_SOURCE_SIZE + REVERB_BUFFER_SIZE).min(ram_slice.len());
    }

    info!("SDRAM ready!");

    let interface = board.audio_peripherals.prepare_interface(Default::default()).await;
    interrupt::SAI1.set_priority(Priority::P6);
    let audio_spawner = AUDIO_EXECUTOR.start(interrupt::SAI1);
    unwrap!(audio_spawner.spawn(audio_task(interface)));

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
    usb_config.product = Some("Stable Granular");
    usb_config.serial_number = Some("STAB01");
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
    (x as i32) as f32 / 8_388_607.0
}
