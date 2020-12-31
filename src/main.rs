#![no_main]
#![no_std]

use embedded_hal::digital::v2::*;
use rtic::app;
use rtic::cyccnt::{Duration, U32Ext};
use rtt_target::rprintln;
use stm32f1xx_hal::{gpio::*, prelude::*};

const CYCLES_PER_STEP: u32 = 200_000;
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
        let dp = cx.device;

        // Enable cycle counter; used for scheduling.
        cp.DWT.enable_cycle_counter();

        // Freeze clock configuration.
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
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

        // Prevent wait-for-interrupt (default rtic idle) from stalling debug features.
        //
        // See: https://github.com/probe-rs/probe-rs/issues/350
        dp.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });
        let _dma1 = dp.DMA1.split(&mut rcc.ahb);

        rprintln!("rtic init completed");

        init::LateResources { led, button, scope }
    }

    #[task(resources = [steps, led, scope], schedule = [blink_led])]
    fn blink_led(cx: blink_led::Context) {
        let blink_led::Resources {
            mut steps,
            led,
            scope,
        } = cx.resources;

        led.toggle().unwrap();
        scope.toggle().unwrap();

        // Schedule next blink.
        let steps = steps.lock(|s| *s);
        let delay = Duration::from_cycles(CYCLES_PER_STEP * steps);
        cx.schedule.blink_led(cx.scheduled + delay).unwrap();
    }

    #[task(binds = EXTI15_10, priority = 2, resources = [button, steps])]
    fn button_press(cx: button_press::Context) {
        rprintln!("button pressed");

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
        fn SPI1();
        fn SPI2();
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {}
}
