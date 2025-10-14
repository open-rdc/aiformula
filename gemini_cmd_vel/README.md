# gemini_cmd_vel

This ROS 2 Python package calls Google Gemini with images from `/image_raw` and
publishes the resulting velocity command on `/cmd_vel`.

## Usage

1. Export your Gemini API key (replace the value with your own):
   ```bash
   export GEMINI_API_KEY="AIzaSy..."
   ```
2. Build the workspace and source the overlay:
   ```bash
   colcon build --packages-select gemini_cmd_vel
   . install/setup.bash
   ```
3. Launch the simulator as usual. The launch file passes the API key to the node
   automatically through the `gemini_api_key` argument. The node logs a warning
   and idles if no key is supplied。モデルを変える場合は
   `ros2 launch simulator gazebo_ignition.launch.py gemini_model:=gemini-flash-latest`
   のように短い名前を渡せば OK です（`models/...` を付けても自動で正規化されます）。

### Customisation

Parameters you may want to adjust:
- `model`: Gemini model name (default: `gemini-flash-latest`).
- `request_interval_sec`: seconds between Gemini calls (default: `8.0`).
- `control_period_sec`: rate (seconds) at which the node republishes smoothed commands (default: `0.1`).
- `max_linear_slew` / `max_angular_slew`: maximum change per second applied during smoothing (defaults: `0.4` / `0.8`).
- `min_linear_speed`: smallest forward speed enforced when a positive command is requested (default: `0.1`).
- `stop_on_failure`: publish zero velocity when the API call fails or response is invalid (default: `true`).
- `failsafe_timeout_sec`: maximum seconds without a fresh command before sending a zero-velocity safety command (default: `10.0`).
- `allow_reverse`: permit negative `linear_x` values from the model (default: `false`).
- `smoothing_window`: number of recent Gemini commands to average before generating a target twist (default: `3`).
- `max_linear_speed` / `max_angular_speed`: clamps for the output `Twist`.
- `base_prompt`: natural-language instructions sent with each request.
