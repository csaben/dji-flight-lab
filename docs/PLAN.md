Drone Autopilot Project: Requirements & Plan
Nov 7, 2026 · @Clark Saben
Goal
Pick five or six points on a 3D map in a loop that ends back home, and the DJI Mini 3 flies that course on its own, steering round anything in the way.
• V1: follow a pre-planned route with GPS waypoints. A camera-based local model takes over near obstacles, then hands back to the route.
• End state: the logged flight path becomes a clean route that plain DJI controls can fly fast, with no neural network in the loop. The safety layer stays on regardless.
• Long term: no GPS. The drone gets a mini map plus a start and end point, and the mini map shrinks and refreshes as it flies.
Hardware, inputs and outputs
The model sees camera frames (plus the downward sensor if the SDK exposes it) and outputs the same stick commands a remote would send.
Item
Detail
Status
Drone
DJI Mini 3, controlled through the DJI SDK
Have it
Camera feed
Encoded by an image VAE into a latent, like the one comma.ai used for self-driving
SDK access likely
Downward (vertical) sensor
Extra input for height and ground
Unconfirmed in SDK
Goal text
Text conditioning on the objective
Optional
Mini map + start/end points
Replaces GPS in the long-term version
Later
Output
Virtual-stick commands, same as the manual controller
Planned
System architecture
The route is planned with plain rules before take-off. A camera safety check runs on every frame and overrides everything else.
After a safety stop, a person resumes the route, takes manual control, or hands off to the local model. The model also corrects the roughly 2-foot error in map positions.
Training
The local model trains in a Gaussian-splat copy of your neighbourhood, then flies the real one.
1. Film the surrounding area. This time it's collected by hand. Later, an automatic "fly around and film" step handles new places.
2. Turn the footage into a Gaussian splat with the World Labs API.
3. Build a local coordinate space from the splat where you can check for collisions.
4. Train with reinforcement learning in that space. Reward shaping keeps the drone moving forward from its starting heading, not turning round.
5. Retrain or add new areas as you go. The current model is tied to the area around your house.
Constraints
• One area first: training covers your house and neighbourhood only.
• Map precision: satellite/3D-map positions are good to about plus or minus 2 feet, not inches. The drone has to correct its own position up close.
• Real-world surprises: wind gusts and birds mean the safety layer stays on, even on routes that are already learned.
• Momentum: at speed, a hover command doesn't stop the drone in time. Braking needs backward thrust.
• SDK limits: what the DJI SDK exposes for the Mini 3 decides which inputs and controls are possible.
Feedback and build order
The biggest risk is SDK access, so confirm it before writing anything else. Then build the safety layer before the model.
Risks
• SDK support: confirm that DJI Mobile SDK v5 supports your exact Mini 3 variant for virtual sticks and live video. Assume the downward sensor isn't exposed until proven otherwise.
• No forward obstacle sensors: the standard Mini 3 has none, so the safety check is camera-only. Measure the camera → phone → model → stick latency, because it sets how far ahead obstacles must be detected.
• Splats aren't solid geometry: convert the splat to a voxel grid or mesh for collision checks.
• Practice-to-real gap: splats look rough away from where you filmed. Capture from many heights and angles, and expect to tune on the real drone.
• Regulations: check FAA rules. Visual line of sight probably applies, which fits the human approve/override step.
Suggested build order
[ ] Prove SDK access: video stream, virtual sticks, telemetry, downward sensor
[ ] Virtual controller with manual override
[ ] Cesium web app: place waypoints, plan around buildings with plain rules
[ ] Fly the planned route with standard DJI waypoints and log the path
[ ] Camera-based safety stop with backward-thrust braking and a human resume/override
[ ] Capture the neighbourhood, build the splat, set up collision checks
[ ] Train the local model with RL and gate it in after a safety stop
[ ] Turn logged flights into a fast route with no model in the loop
[ ] Research track: mini-map navigation without GPS, starting from a simple goal direction
Open questions
• Which Mini 3 variant is it (Mini 3 or Mini 3 Pro)? The Pro has forward sensors.
• After a safety stop, who decides between resume, manual and Roomba mode: always the human, or the model by default?
