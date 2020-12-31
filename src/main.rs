#![no_main]
#![no_std]

use embedded_hal::digital::v2::*;
use pac::interrupt;
use rtic::app;
use rtic::cyccnt::{Duration, U32Ext};
use rtt_target::rprintln;
use stm32f1xx_hal::{gpio::*, pac, prelude::*};

const CYCLES_PER_STEP: u32 = 80_000;
const MAX_STEPS: u32 = 10;

#[app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        #[init(0)]
        steps: u32,
        led: gpioc::PC13<Output<PushPull>>,
        button: gpioa::PA10<Input<PullUp>>,
        scope: gpioa::PA4<Output<PushPull>>,
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
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        rprintln!("sysclk: {:?} MHz", clocks.sysclk().0 / 1_000_000);

        let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
        let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
        let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

        // Configure pc13 as output via CR high register.
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high().unwrap(); // LED off

        // Configure pa4 as output for oscilloscope.
        let mut scope = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        scope.set_low().unwrap(); // Oscill low

        // Setup pa10 button interrupt.
        let mut button = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);
        button.make_interrupt_source(&mut afio);
        button.trigger_on_edge(&dp.EXTI, Edge::FALLING);
        button.enable_interrupt(&dp.EXTI);

        cx.schedule
            .blink_led(cx.start + CYCLES_PER_STEP.cycles())
            .unwrap();

        rprintln!("rtic init completed");

        init::LateResources { led, button, scope }
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

    #[task(resources = [steps, led, scope], schedule = [blink_led])]
    fn blink_led(mut cx: blink_led::Context) {
        cx.resources.led.toggle().unwrap();
        cx.resources.scope.toggle().unwrap();
        let steps = cx.resources.steps.lock(|steps| *steps);

        // Schedule next blink.
        let delay = Duration::from_cycles(CYCLES_PER_STEP * steps);
        cx.schedule.blink_led(cx.scheduled + delay).unwrap();
    }

    #[task(binds = EXTI15_10, priority = 2, resources = [button, steps])]
    fn button_press(cx: button_press::Context) {
        let button = cx.resources.button;
        if !button.check_interrupt() {
            return;
        }
        button.clear_interrupt_pending_bit();

        let steps = *cx.resources.steps;
        let new_steps = (steps % MAX_STEPS) + 1;
        *cx.resources.steps = new_steps;

        rprintln!("steps: {} -> {}", steps, new_steps);
    }

    // Unused interrupts for task scheduling.
    extern "C" {
        fn EXTI0();
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {}
}
