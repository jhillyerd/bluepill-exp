#![no_main]
#![no_std]

use debouncr::Debouncer;
use embedded_hal::digital::v2::*;
use rtic::app;
use rtic::cyccnt::{Duration, U32Ext};
use rtt_target::rprintln;
use stm32f1xx_hal::{gpio::*, pac, prelude::*, pwm, rcc::Clocks, timer};

const SYSCLK_HZ: u32 = 72_000_000;
const BUTTON_POLL_PERIOD: u32 = SYSCLK_HZ / 100;
const CYCLES_PER_STEP: u32 = 1_000_000;
const MAX_STEPS: u32 = 10;
const PWM_LEVELS: [u16; 8] = [0, 5, 10, 15, 25, 40, 65, 100];

#[app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        #[init(1)]
        steps: u32,
        #[init(0)]
        pwm_level: usize,
        tim2: timer::CountDownTimer<pac::TIM2>,
        led: gpioc::PC13<Output<PushPull>>,
        button: gpioa::PA10<Input<PullUp>>,
        button_state: Debouncer<u8, debouncr::Repeat6>,
        scope: gpioa::PA4<Output<PushPull>>,
        led_pwm: pwm::Pwm<pac::TIM3, timer::Tim3NoRemap, pwm::C1, gpioa::PA6<Alternate<PushPull>>>,
    }

    #[init(spawn = [blink_led, poll_button])]
    fn init(ctx: init::Context) -> init::LateResources {
        rtt_target::rtt_init_print!();
        rprintln!("RTIC init started");
        let mut cp = ctx.core;
        let dp = ctx.device;

        // Enable cycle counter; used for scheduling.
        cp.DWT.enable_cycle_counter();

        // Setup and apply clock confiugration.
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let clocks: Clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(SYSCLK_HZ.hz())
            .pclk1((SYSCLK_HZ / 2).hz())
            .freeze(&mut flash.acr);
        rprintln!(" SYSCLK: {:?} MHz", clocks.sysclk().0 / 1_000_000);
        rprintln!(" HCLK: {:?} MHz", clocks.hclk().0 / 1_000_000);
        rprintln!(" APB1 clk: {:?} MHz", clocks.pclk1().0 / 1_000_000);
        rprintln!(" APB1 TIM: {:?} MHz", clocks.pclk1_tim().0 / 1_000_000);
        rprintln!(" APB2 clk: {:?} MHz", clocks.pclk2().0 / 1_000_000);
        rprintln!(" ADCCLK: {:?} MHz", clocks.adcclk().0 / 1_000_000);

        // Timer setup.
        let mut tim2: timer::CountDownTimer<pac::TIM2> =
            timer::Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(2.khz());
        tim2.listen(timer::Event::Update);

        // Peripheral setup.
        let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
        let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
        let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

        // Configure pc13 as output via CR high register.
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high().unwrap(); // LED off

        // Configure pa4 as output for oscilloscope.
        let mut scope = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        scope.set_low().unwrap(); // Oscill low

        // Setup pa10 button.
        let button = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

        // Setup TIM3 PWM CH1 on PA6.
        let pa6 = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);
        let pwm_pins = pa6;
        let mut led_pwm = timer::Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1).pwm(
            pwm_pins,
            &mut afio.mapr,
            1.khz(),
        );
        led_pwm.set_duty(pwm::Channel::C1, 0);
        led_pwm.enable(pwm::Channel::C1);

        ctx.spawn.poll_button().unwrap();
        ctx.spawn.blink_led().unwrap();

        // Prevent wait-for-interrupt (default rtic idle) from stalling debug features.
        //
        // See: https://github.com/probe-rs/probe-rs/issues/350
        dp.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });
        let _dma1 = dp.DMA1.split(&mut rcc.ahb);

        rprintln!("RTIC init completed");

        init::LateResources {
            led,
            tim2,
            button,
            button_state: debouncr::debounce_6(false),
            scope,
            led_pwm,
        }
    }

    #[task(resources = [steps, led], schedule = [blink_led])]
    fn blink_led(ctx: blink_led::Context) {
        let blink_led::Resources { mut steps, led } = ctx.resources;

        led.toggle().unwrap();

        // Schedule next blink.
        let steps = steps.lock(|s| *s);
        let delay = Duration::from_cycles(CYCLES_PER_STEP * steps);
        ctx.schedule.blink_led(ctx.scheduled + delay).unwrap();
    }

    #[task(binds = TIM2, priority = 3, resources = [tim2, scope])]
    fn toggle_scope(ctx: toggle_scope::Context) {
        let toggle_scope::Resources { tim2, scope } = ctx.resources;

        scope.toggle().unwrap();
        tim2.clear_update_interrupt_flag();
    }

    #[task(
        priority = 2,
        resources = [button, button_state],
        spawn = [button_press],
        schedule = [poll_button]
    )]
    fn poll_button(ctx: poll_button::Context) {
        // Button is active low.
        let pressed = ctx.resources.button.is_low().unwrap();
        let edge = ctx.resources.button_state.update(pressed);
        if edge == Some(debouncr::Edge::Rising) {
            ctx.spawn.button_press().unwrap();
        }

        // Schedule next button poll.
        ctx.schedule
            .poll_button(ctx.scheduled + BUTTON_POLL_PERIOD.cycles())
            .unwrap();
    }

    #[task(priority = 2, resources = [steps, pwm_level], spawn = [update_led_pwm])]
    fn button_press(ctx: button_press::Context) {
        let button_press::Resources { steps, pwm_level } = ctx.resources;

        // Increment steps.
        let old_steps = *steps;
        *steps = (old_steps % MAX_STEPS) + 1;
        rprintln!("steps: {} -> {}", old_steps, steps);

        // Rotate pwm_level.
        let old_pwm_level = *pwm_level;
        *pwm_level = (old_pwm_level + 1) % PWM_LEVELS.len();
        ctx.spawn.update_led_pwm().unwrap();
    }

    #[task(resources = [pwm_level, led_pwm])]
    fn update_led_pwm(ctx: update_led_pwm::Context) {
        let update_led_pwm::Resources {
            mut pwm_level,
            led_pwm,
        } = ctx.resources;
        let pwm_level = pwm_level.lock(|v| *v);

        let max_duty = led_pwm.get_max_duty();
        let duty = max_duty / 100 * PWM_LEVELS[pwm_level];
        led_pwm.set_duty(pwm::Channel::C1, duty);
        rprintln!(
            "led_pwm duty = {}% ({} / {})",
            PWM_LEVELS[pwm_level],
            duty,
            max_duty
        );
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
