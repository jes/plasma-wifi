//! Per-axis tap destinations and temporary held-key travel. All state is owned
//! by the controller worker; UI frames submit input, never mutate this model.
use crate::gcode::Point;
use std::time::{Duration, Instant};

pub const HOLD_DELAY: Duration = Duration::from_millis(150);
const EPS: f64 = 0.001;

pub struct Axis {
    pub incremental: f64,
    pub target: f64,
    pub met: bool,
    pub direction: i8,
    pub pressed: Option<Instant>,
    last: f64,
    last_motion: Instant,
    last_input: Instant,
    discard_held: bool,
    after_stop_steps: f64,
}
impl Axis {
    fn new(position: f64, now: Instant) -> Self {
        Self {
            incremental: position,
            target: position,
            met: true,
            direction: 0,
            pressed: None,
            last: position,
            last_motion: now,
            last_input: now,
            discard_held: false,
            after_stop_steps: 0.0,
        }
    }
    pub fn held(&self, now: Instant) -> bool {
        self.direction != 0
            && self
                .pressed
                .is_some_and(|at| now.duration_since(at) >= HOLD_DELAY)
    }
    fn observe(&mut self, position: f64, now: Instant) {
        if (position - self.last).abs() > EPS {
            self.last_motion = now;
        }
        if (position - self.incremental).abs() <= EPS
            || (position - self.incremental) * (self.last - self.incremental) < 0.0
        {
            self.met = true;
        }
        self.last = position;
        if self.direction == 0
            && self.met
            && now.duration_since(self.last_motion) >= Duration::from_millis(500)
            && now.duration_since(self.last_input) >= Duration::from_millis(500)
        {
            self.incremental = position;
            self.target = position;
        }
    }
    fn tap(&mut self, count: i32, step: f64, now: Instant) {
        if count == 0 {
            return;
        }
        if self.discard_held {
            self.after_stop_steps += f64::from(count) * step;
            self.last_input = now;
            return;
        }
        if self.met && (self.target - self.last).abs() < EPS {
            self.incremental = self.last;
        }
        self.incremental += f64::from(count) * step;
        let old = self.target - self.last;
        let new = self.incremental - self.last;
        // An incremental destination already passed by held travel must not
        // drag the axis back. An unfinished destination still accumulates taps.
        if !self.met || old.abs() < EPS || old * new >= 0.0 && new.abs() > old.abs() {
            self.target = self.incremental;
            self.met = new.abs() <= EPS;
        }
        self.last_input = now;
    }
    fn input(&mut self, direction: i8, now: Instant) -> bool {
        if direction == self.direction {
            return false;
        }
        let cancel = self.held(now);
        if cancel {
            // Once a press became a hold, release means stop this axis, even
            // if its initial increment has not been reached. Fresh taps that
            // arrive during braking are based on the stopped position.
            self.discard_held = true;
            self.after_stop_steps = 0.0;
        }
        if self.met
            && direction != 0
            && (self.direction != 0 || (self.target - self.last).abs() < EPS)
        {
            self.incremental = self.last;
            self.target = self.last;
        }
        self.direction = direction;
        self.pressed = (direction != 0).then_some(now);
        self.last_input = now;
        cancel
    }
    fn rebase(&mut self, current: f64, now: Instant) {
        self.observe(current, now);
        if self.discard_held {
            self.incremental = current + self.after_stop_steps;
            self.target = self.incremental;
            self.met = self.after_stop_steps.abs() < EPS;
            self.discard_held = false;
            self.after_stop_steps = 0.0;
        }
        if self.direction == 0 {
            self.target = if self.met { current } else { self.incremental };
        } else if (self.target - current) * f64::from(self.direction) < 0.0 {
            self.target = current;
        }
    }
}

pub struct ManualJog {
    pub axes: [Axis; 2],
    pub speed: f64,
    pub active_speed: f64,
    pub cursor: Point,
    pub target: Point,
    pub heartbeat: Instant,
    pub extended: Instant,
    pub sent: bool,
    pub sent_at: Option<Instant>,
}
impl ManualJog {
    pub fn new(position: Point, speed: f64, now: Instant) -> Self {
        Self {
            axes: [Axis::new(position.x, now), Axis::new(position.y, now)],
            speed,
            active_speed: speed,
            cursor: position,
            target: position,
            heartbeat: now,
            extended: now,
            sent: false,
            sent_at: None,
        }
    }
    pub fn directions(&self) -> [i8; 2] {
        [self.axes[0].direction, self.axes[1].direction]
    }
    pub fn released(&self) -> bool {
        self.directions() == [0, 0]
    }
    pub fn observe(&mut self, position: Point, now: Instant) {
        self.axes[0].observe(position.x, now);
        self.axes[1].observe(position.y, now);
        self.sync_target();
    }
    pub fn input(
        &mut self,
        directions: [i8; 2],
        presses: [i32; 2],
        step: f64,
        now: Instant,
    ) -> bool {
        let mut cancel = false;
        for i in 0..2 {
            // Sample whether the incremental destination has been reached
            // before changing held flags or accumulating another press.
            cancel |= self.axes[i].input(directions[i], now);
            self.axes[i].tap(presses[i], step, now);
        }
        self.heartbeat = now;
        self.sync_target();
        cancel || presses != [0, 0] && self.sent
    }
    pub fn go_to(&mut self, target: Point) {
        for (axis, value) in self.axes.iter_mut().zip([target.x, target.y]) {
            axis.incremental = value;
            axis.target = value;
            axis.met = false;
            axis.direction = 0;
            axis.pressed = None;
        }
        self.sync_target();
    }
    pub fn rebase(&mut self, current: Point, now: Instant) {
        self.axes[0].rebase(current.x, now);
        self.axes[1].rebase(current.y, now);
        self.cursor = current;
        self.sent = true;
        self.sent_at = None;
        self.active_speed = self.speed;
        self.sync_target();
    }
    pub fn held_direction(&self, now: Instant) -> Point {
        Point {
            x: if self.axes[0].held(now) {
                f64::from(self.axes[0].direction)
            } else {
                0.0
            },
            y: if self.axes[1].held(now) {
                f64::from(self.axes[1].direction)
            } else {
                0.0
            },
            z: 0.0,
        }
    }
    pub fn extend(&mut self, current: Point, distance: f64, now: Instant) {
        let direction = self.held_direction(now);
        let length = direction.x.hypot(direction.y);
        if length < 1e-9 {
            return;
        }
        for (axis, position, component) in self
            .axes
            .iter_mut()
            .zip([current.x, current.y])
            .zip([direction.x, direction.y])
            .map(|((a, p), c)| (a, p, c))
        {
            if component != 0.0 {
                let target = position + distance * component / length;
                if (target - axis.target) * component > 0.0 {
                    axis.target = target;
                }
            }
        }
        self.extended = now;
        self.sync_target();
    }
    fn sync_target(&mut self) {
        self.target = Point {
            x: self.axes[0].target,
            y: self.axes[1].target,
            z: self.cursor.z,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn p(x: f64, y: f64) -> Point {
        Point { x, y, z: 0.0 }
    }
    #[test]
    fn taps_accumulate_without_position_feedback_and_diagonals_step_each_axis() {
        let now = Instant::now();
        let mut jog = ManualJog::new(p(10.0, 20.0), 600.0, now);
        jog.input([0, 0], [2, 1], 1.0, now);
        assert_eq!(jog.target, p(12.0, 21.0));
        jog.input([0, 0], [-1, 0], 1.0, now);
        assert_eq!(jog.target, p(11.0, 21.0));
    }
    #[test]
    fn held_release_discards_the_released_axis_and_keeps_other_held_axes() {
        let now = Instant::now();
        let later = now + HOLD_DELAY;
        let mut jog = ManualJog::new(p(0.0, 0.0), 600.0, now);
        jog.input([1, 1], [1, 1], 1.0, now);
        jog.observe(p(2.0, 0.5), later);
        jog.extend(p(2.0, 0.5), 5.0, later);
        assert!(jog.input([0, 1], [0, 0], 1.0, later));
        jog.rebase(p(2.4, 0.8), later);
        assert_eq!(jog.target.x, 2.4);
        assert!(jog.target.y > 1.0);
        jog.input([0, 0], [0, 0], 1.0, later);
        jog.rebase(p(2.4, 0.9), later);
        assert_eq!(jog.target, p(2.4, 0.9));
    }
    #[test]
    fn held_release_discards_an_unfinished_large_initial_step() {
        let now = Instant::now();
        let mut jog = ManualJog::new(p(0.0, 0.0), 600.0, now);
        jog.input([1, 0], [1, 0], 100.0, now);
        let later = now + HOLD_DELAY;
        assert!(jog.input([0, 0], [0, 0], 100.0, later));
        jog.rebase(p(0.7, 0.0), later);
        assert_eq!(jog.target, p(0.7, 0.0));
        assert!(jog.axes[0].met);
    }
    #[test]
    fn fresh_taps_during_braking_use_the_final_position_and_other_axis_taps_survive() {
        let now = Instant::now();
        let mut jog = ManualJog::new(p(0.0, 0.0), 600.0, now);
        jog.input([1, 0], [1, 1], 100.0, now);
        let later = now + HOLD_DELAY;
        jog.input([0, 0], [0, 0], 100.0, later);
        jog.input([0, 0], [-1, 0], 2.0, later + Duration::from_millis(10));
        jog.rebase(p(0.7, 0.5), later + Duration::from_millis(20));
        assert_eq!(jog.target, p(-1.3, 100.0));
    }

    #[test]
    fn elapsed_time_detects_holds_without_os_key_repeat_and_taps_finish() {
        let now = Instant::now();
        let mut jog = ManualJog::new(p(0.0, 0.0), 600.0, now);
        jog.input([1, 0], [1, 0], 1.0, now);
        assert_eq!(jog.held_direction(now).x, 0.0);
        assert_eq!(jog.held_direction(now + HOLD_DELAY).x, 1.0);
        assert!(!jog.input([0, 0], [0, 0], 1.0, now + Duration::from_millis(50)));
        jog.rebase(p(0.25, 0.0), now + Duration::from_millis(50));
        assert_eq!(jog.target.x, 1.0);
    }
    #[test]
    fn a_fast_tap_after_held_release_starts_at_the_stopped_position() {
        let now = Instant::now();
        let later = now + HOLD_DELAY;
        let mut jog = ManualJog::new(p(0.0, 0.0), 600.0, now);
        jog.input([1, 0], [1, 0], 1.0, now);
        jog.observe(p(3.0, 0.0), later);
        jog.extend(p(3.0, 0.0), 5.0, later);
        jog.input([0, 0], [0, 0], 1.0, later);
        jog.rebase(p(3.5, 0.0), later);
        jog.input([0, 0], [1, 0], 1.0, later + Duration::from_millis(20));
        assert_eq!(jog.target.x, 4.5);
    }

    #[test]
    fn crossing_increment_and_stationary_grace_reset_do_not_pull_back() {
        let now = Instant::now();
        let mut jog = ManualJog::new(p(0.0, 0.0), 600.0, now);
        jog.input([1, 0], [1, 0], 1.0, now);
        let later = now + HOLD_DELAY;
        jog.observe(p(3.0, 0.0), later);
        jog.extend(p(3.0, 0.0), 5.0, later);
        jog.input([1, 0], [1, 0], 1.0, later);
        assert!(jog.target.x >= 8.0);
        jog.input([0, 0], [0, 0], 1.0, later);
        jog.rebase(p(3.5, 0.0), later);
        assert_eq!(jog.target.x, 3.5);
        jog.observe(p(3.5, 0.0), later + Duration::from_secs(1));
        assert_eq!(jog.axes[0].incremental, 3.5);
        jog.input([0, 0], [1, 0], 1.0, later + Duration::from_secs(1));
        assert_eq!(jog.target.x, 4.5);
    }
}
