/*
 * Differential-drive recruitment task
 *
 * The communication and decoding stages provide a target coordinate. Implement
 * drive_to_target() so the simulated differential-drive rover reaches the
 * target using valid left and right wheel velocities.
 */

#include <math.h>
#include <stdbool.h>

#define PI_F             3.14159265358979323846f

#define WHEEL_RADIUS      0.15f
#define WHEEL_SEPARATION  0.77f
#define MAX_LINEAR_VELOCITY  1.0f
#define MAX_ANGULAR_VELOCITY 2.0f
#define MAX_WHEEL_VELOCITY  10.0f
#define HEADING_GAIN      1.25f

#define TARGET_TOLERANCE  0.10f
#define DRIVE_DT_SECONDS  0.02f
#define MAX_DRIVE_STEPS   6000

/*
 * Latitude and longitude are normalized local simulation coordinates measured
 * in metres. Latitude is the north axis and longitude is the east axis. The
 * differential-drive rover is planar, so altitude is received but not changed.
 */
struct coordinate {
  float latitude;
  float longitude;
  float altitude;
};

/* Heading is in radians: zero points east and positive rotation is CCW. */
struct rover_state {
  struct coordinate position;
  float heading_rad;
};

struct wheel_velocity {
  float left;
  float right;
};

enum drive_status {
  DRIVE_REACHED_TARGET   =  0,
  DRIVE_INVALID_INPUT    = -1,
  DRIVE_INVALID_COMMAND  = -2,
  DRIVE_MAX_STEPS_EXCEEDED = -3
};

/* Provided simulator helpers. Candidates should not modify these functions. */
static float normalize_angle(float angle);
static bool  apply_wheel_velocities(struct rover_state *rover,
                                     struct wheel_velocity velocity);

/*
 * Candidate task — drive_to_target()
 * ------------------------------------
 * Proportional heading controller with cosine-scaled linear velocity.
 *
 * Algorithm per time-step:
 *  1. Validate rover and target (null + finite check).
 *  2. Compute (dx=east, dy=north) displacement to target.
 *  3. Stop when Euclidean distance <= TARGET_TOLERANCE.
 *  4. desired_heading = atan2f(north, east)  [matches rover convention].
 *  5. heading_error   = normalize_angle(desired - current).
 *  6. angular_vel     = HEADING_GAIN * error  (clamped to MAX_ANGULAR_VELOCITY).
 *  7. linear_vel      = MAX_LINEAR * cos(error) clamped to [0, MAX_LINEAR].
 *       - When aligned (error=0)   : full speed forward.
 *       - When 90° off (error=pi/2): zero forward, pure rotation.
 *       - Smooth arc in between.
 *  8. Convert to individual wheel velocities (differential-drive kinematics):
 *       v_left  = (linear - angular * L/2) / R
 *       v_right = (linear + angular * L/2) / R
 *  9. Clamp each wheel to [-MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY].
 * 10. Apply via simulator helper; return DRIVE_INVALID_COMMAND on rejection.
 * 11. Return DRIVE_MAX_STEPS_EXCEEDED if target not reached within budget.
 */
enum drive_status drive_to_target(struct rover_state *rover,
                                   const struct coordinate *target) {
  /* Step 1 – Validate inputs */
  if (rover == NULL || target == NULL) {
    return DRIVE_INVALID_INPUT;
  }
  if (!isfinite(rover->position.latitude)  ||
      !isfinite(rover->position.longitude) ||
      !isfinite(rover->heading_rad)         ||
      !isfinite(target->latitude)           ||
      !isfinite(target->longitude)) {
    return DRIVE_INVALID_INPUT;
  }

  int step;
  for (step = 0; step < MAX_DRIVE_STEPS; step++) {

    /* Step 2 – Direction vector (east=longitude axis, north=latitude axis) */
    float dx = target->longitude - rover->position.longitude;  /* east  */
    float dy = target->latitude  - rover->position.latitude;   /* north */

    /* Step 3 – Distance check */
    float distance = hypotf(dx, dy);
    if (distance <= TARGET_TOLERANCE) {
      return DRIVE_REACHED_TARGET;
    }

    /* Step 4 – Desired heading (zero=east, CCW positive — matches atan2) */
    float desired_heading = atan2f(dy, dx);

    /* Step 5 – Heading error with wraparound */
    float heading_error = normalize_angle(desired_heading - rover->heading_rad);

    /* Step 6 – Angular velocity proportional to error, clamped */
    float angular_vel = HEADING_GAIN * heading_error;
    if (angular_vel >  MAX_ANGULAR_VELOCITY) angular_vel =  MAX_ANGULAR_VELOCITY;
    if (angular_vel < -MAX_ANGULAR_VELOCITY) angular_vel = -MAX_ANGULAR_VELOCITY;

    /* Step 7 – Linear velocity: cosine-scaled so the rover arcs smoothly.
     * cos(0)    = 1.0  -> full forward when aligned.
     * cos(pi/2) = 0.0  -> pure rotation when 90 degrees off.
     * Clamped to [0, MAX] so we never drive backwards during a turn.
     */
    float cos_err   = cosf(heading_error);
    float linear_vel = MAX_LINEAR_VELOCITY * (cos_err > 0.0f ? cos_err : 0.0f);

    /* Step 8 – Differential-drive kinematics
     *   v_left  = (V_linear - omega * L/2) / R
     *   v_right = (V_linear + omega * L/2) / R
     */
    float half_sep = WHEEL_SEPARATION / 2.0f;
    float v_left   = (linear_vel - angular_vel * half_sep) / WHEEL_RADIUS;
    float v_right  = (linear_vel + angular_vel * half_sep) / WHEEL_RADIUS;

    /* Step 9 – Clamp to maximum wheel velocity */
    if (v_left  >  MAX_WHEEL_VELOCITY) v_left  =  MAX_WHEEL_VELOCITY;
    if (v_left  < -MAX_WHEEL_VELOCITY) v_left  = -MAX_WHEEL_VELOCITY;
    if (v_right >  MAX_WHEEL_VELOCITY) v_right =  MAX_WHEEL_VELOCITY;
    if (v_right < -MAX_WHEEL_VELOCITY) v_right = -MAX_WHEEL_VELOCITY;

    /* Step 10 – Apply via simulator helper */
    struct wheel_velocity vel = {v_left, v_right};
    if (!apply_wheel_velocities(rover, vel)) {
      return DRIVE_INVALID_COMMAND;
    }
  }

  /* Step 11 – Budget exhausted */
  return DRIVE_MAX_STEPS_EXCEEDED;
}

static float normalize_angle(float angle) {
  while (angle >  PI_F) { angle -= 2.0f * PI_F; }
  while (angle < -PI_F) { angle += 2.0f * PI_F; }
  return angle;
}

static bool apply_wheel_velocities(struct rover_state *rover,
                                    struct wheel_velocity velocity) {
  if (!isfinite(velocity.left)  || !isfinite(velocity.right) ||
      fabsf(velocity.left)  > MAX_WHEEL_VELOCITY ||
      fabsf(velocity.right) > MAX_WHEEL_VELOCITY) {
    return false;
  }

  const float linear_velocity =
      WHEEL_RADIUS * (velocity.left + velocity.right) / 2.0f;
  const float angular_velocity =
      WHEEL_RADIUS * (velocity.right - velocity.left) / WHEEL_SEPARATION;

  rover->heading_rad = normalize_angle(
      rover->heading_rad + angular_velocity * DRIVE_DT_SECONDS);
  rover->position.longitude +=
      linear_velocity * cosf(rover->heading_rad) * DRIVE_DT_SECONDS;
  rover->position.latitude  +=
      linear_velocity * sinf(rover->heading_rad) * DRIVE_DT_SECONDS;

  return true;
}