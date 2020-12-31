#![no_main]
#![no_std]

use embedded_hal::digital::v2::*;
// use pac::interrupt;
use rtic::app;
use rtic::cyccnt::{Duration, U32Ext};
use rtt_target::rprintln;
use stm32f1xx_hal::{gpio::*, pac, prelude::*};

const CYCLES_PER_STEP: u32 = 80_000;
const _MAX_STEPS: usize = 10;

#[app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        #[init(0)]
        steps: usize,
        led: gpioc::PC13<Output<PushPull>>,
    }

    #[init(schedule = [blink_led])]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_target::rtt_init_print!();
        rprintln!("rtic init started");

        let mut cp = cx.core;

        // Enable cycle counter; used for scheduling.
        cp.DWT.enable_cycle_counter();

        let dp = cx.device;
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();

        // Freeze clock configuration.
        let _clocks = rcc.cfgr.freeze(&mut flash.acr);

        let mut _afio = dp.AFIO.constrain(&mut rcc.apb2);
        // let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
        let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

        // Configure pc13 as output via CR high register.
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high().unwrap(); // LED off

        // let mut pa4 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

        // pa4.set_low().unwrap(); // Oscill low

        // // Setup pa10 button interrupt.  Safety: Will not be accessed from anywhere but the interrupt
        // // handler after we initialize.
        // {
        //     let button = unsafe { &mut *BUTTON.as_mut_ptr() };
        //     *button = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);
        //     button.make_interrupt_source(&mut afio);
        //     button.trigger_on_edge(&dp.EXTI, Edge::FALLING);
        //     button.enable_interrupt(&dp.EXTI);
        // }

        // unsafe {
        //     // Pins 10-15 are on the 15-10 EXTI.
        //     pac::NVIC::unmask(pac::Interrupt::EXTI15_10);
        // }

        cx.schedule
            .blink_led(cx.start + CYCLES_PER_STEP.cycles())
            .unwrap();

        rprintln!("rtic init completed");

        init::LateResources { led }
    }

    /// Define an idle function to prevent wait-for-interrupt from blocking the debugger.
    ///
    /// See: https://github.com/probe-rs/probe-rs/issues/350
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            core::sync::atomic::spin_loop_hint();
        }
    }

    #[task(resources = [led], schedule = [blink_led])]
    fn blink_led(cx: blink_led::Context) {
        cx.resources.led.toggle().unwrap();

        cx.schedule
            .blink_led(cx.scheduled + Duration::from_cycles(CYCLES_PER_STEP * 5))
            .unwrap();

        // TODO based on steps
    }

    // Unused interrupts for task scheduling.
    extern "C" {
        fn EXTI0();
    }
};

// #[interrupt]
// fn EXTI15_10() {
//     static mut BUTTON: MaybeUninit<gpioa::PA10<Input<PullUp>>> = MaybeUninit::uninit();
//     let button = unsafe { &mut *BUTTON.as_mut_ptr() };
//     if !button.check_interrupt() {
//         return;
//     }

//     let steps = STEPS.load(Ordering::Relaxed);
//     let new_steps = (steps % MAX_STEPS) + 1;
//     STEPS.compare_and_swap(steps, new_steps, Ordering::Relaxed);
//     button.clear_interrupt_pending_bit();

//     rprintln!("steps: {} -> {}", steps, new_steps);
// }

#[inline(never)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {}
}
