let offset = [0, 0];
let width = innerWidth;
let height = innerHeight;
let scene;

function setup() {
  const canvas = createCanvas(width, height);
  syncWindow();

  scene = new Scene(offset);
}

function windowResized() {
  syncWindow();
}

function syncWindow() {
  width = innerWidth;
  height = innerHeight;
  let aspect = width / height;
  offset = [
    40 * aspect,
    40
  ];

  resizeCanvas(width, height);
}

function draw() {
  scene.offset = offset;
  scene.draw();
}

class Scene {
  constructor(offset) {
    this.offset = offset;
    this.size = [
      20 * 2.54 * 10, // 20 inches
      20.25 * 2.54 * 10, // 20 inches
    ]

    this.path = [
      {
        x: 100,
        y: 100,
        speed: 100,
      },
      {
        x: 200,
        y: 50,
        speed: 100,
      },
      {
        x: 300,
        y: 150,
        speed: 100,
      },
      {
        x: 400,
        y: 100,
        speed: 100,
      },
    ];
    this.current_target_index = 0;

    this.tool_pos = [0, 0];
    this.prev_pos = this.tool_pos;
    this.velocity = [0, 0, 0];
    this.acceleration = [0, 0, 0];

    this.time_step = 0.01;
  }

  draw() {
    background(0, 0, 0);
    translate(this.offset[0], this.offset[1]);
    fill(255, 0, 0);

    this.update();

    this.drawPath();
    this.drawTool();
    this.drawBoundary();

    this.drawVelocity();
    this.drawAcceleration();
  }

  update() {
    if (this.current_target_index < this.path.length) {
      let target = this.path[this.current_target_index];

      // this.tool_pos = [target.x, target.y];
      // lerp to target
      this.move_tool_to(target, target.speed);

      if (dist(this.tool_pos[0], this.tool_pos[1], target.x, target.y) < 1) {
        this.current_target_index++;
      }
    }

    // this.path.shift();
    // this.path.push([
    //   this.tool[0] + 100 * cos(this.time),
    //   this.tool[1] + 100 * sin(this.time)
    // ]);
  }

  move_tool_to(target, speed) {
    let x = this.tool_pos[0];
    let y = this.tool_pos[1];

    let dx = target.x - x;
    let dy = target.y - y;

    let angle = atan2(dy, dx);
    let distance = dist(x, y, target.x, target.y);

    let step = speed * this.time_step;
    let step_x = step * cos(angle);
    let step_y = step * sin(angle);

    [step_x, step_y] = this.clampAcceleration(step_x, step_y);

    if (distance < step) {
      x = target.x;
      y = target.y;
    }
    else {
      x += step_x;
      y += step_y;
    }

    this.tool_pos = [x, y];
  }

  drawArrow(start, vector, color, scale = 50) {
    let [x, y, magnitude] = vector;

    push();
    stroke(color);
    strokeWeight(2);
    fill(color);

    // Translate to the start position
    translate(start[0], start[1]);

    // Draw the line
    line(0, 0, x * scale, y * scale);

    // Calculate the angle of the arrow
    let angle = atan2(y, x);

    // Translate to the endpoint of the line
    translate(x * scale, y * scale);

    // Rotate to align the arrowhead with the line
    rotate(angle);

    // Draw the arrowhead
    let arrowSize = 3;
    translate(-arrowSize, 0); // Move back by the arrow size to align it perfectly
    triangle(0, arrowSize / 2, 0, -arrowSize / 2, arrowSize, 0);

    pop();

  }

  drawVelocity() {
    this.drawArrow(this.tool_pos, this.velocity, color(0, 255, 0));
  }

  drawAcceleration() {
    this.drawArrow(this.tool_pos, this.acceleration, color(0, 0, 255));
  }

  clampAcceleration(step_x, step_y) {
    let max_acceleration = 0.01;
    let max_velocity = 0.5;

    // Calculate desired acceleration
    let desired_acceleration_x = step_x - this.velocity[0];
    let desired_acceleration_y = step_y - this.velocity[1];

    // Compute the magnitude of the desired acceleration
    let desired_magnitude = sqrt(
      desired_acceleration_x ** 2 + desired_acceleration_y ** 2
    );

    // Clamp the acceleration if it exceeds max_acceleration
    let acceleration_x = desired_acceleration_x;
    let acceleration_y = desired_acceleration_y;

    if (desired_magnitude > max_acceleration) {
      acceleration_x = (desired_acceleration_x / desired_magnitude) * max_acceleration;
      acceleration_y = (desired_acceleration_y / desired_magnitude) * max_acceleration;
    }

    // Update the acceleration vector and its magnitude
    this.acceleration = [acceleration_x, acceleration_y, sqrt(acceleration_x ** 2 + acceleration_y ** 2)];

    // Update the velocity based on the clamped acceleration
    let velocity_x = this.velocity[0] + acceleration_x;
    let velocity_y = this.velocity[1] + acceleration_y;

    // Compute the magnitude of the velocity
    let velocity_magnitude = sqrt(velocity_x ** 2 + velocity_y ** 2);

    // Clamp the velocity if it exceeds max_velocity
    if (velocity_magnitude > max_velocity) {
      velocity_x = (velocity_x / velocity_magnitude) * max_velocity;
      velocity_y = (velocity_y / velocity_magnitude) * max_velocity;
    }

    // Update the velocity vector and its magnitude
    this.velocity = [velocity_x, velocity_y, sqrt(velocity_x ** 2 + velocity_y ** 2)];

    // Return the updated velocity vector
    return [velocity_x, velocity_y];
  }


  drawTool() {
    push();
    fill(255, 0, 0);
    circle(this.tool_pos[0], this.tool_pos[1], 10, 10);
    pop();
  }

  drawPath() {
    push();
    beginShape();
    noFill();
    stroke(255);
    for (let path of this.path) {
      vertex(path.x, path.y);
    }
    endShape();
    pop();
  }

  drawBoundary() {
    push();
    stroke(255);
    noFill();
    rect(0, 0, this.size[0], this.size[1]);
    pop();
  }

  drawMotors() {

  }
}