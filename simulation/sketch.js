let width = innerWidth;
let height = innerHeight;

function setup() {
  createCanvas(width, height);
  syncWindow();
}

function windowResized() {
  syncWindow();
}

function syncWindow() {
  width = innerWidth;
  height = innerHeight;

  resizeCanvas(width, height);
}

function mousePressed(a) {
  box_pos_x = (mouseX - width / 2) / 1.8 / 2;
  box_pos_y = (mouseY - height / 2) / 1.8 / 2;

  if (box_pos_x < -100 || box_pos_x > 100 || box_pos_y < -100 || box_pos_y > 100) {
    return;
  }


  window.machine?.writer?.write(new TextEncoder().encode(`go ${box_pos_x} ${box_pos_y} 0 1000\n`));
}

let motion_state = null;

let path = [];

function draw() {
  background(0, 0, 0);

  let boxSize = 200;
  let scaleFactor = (0.8 * height) / boxSize;

  translate(width / 2, height / 2);
  scale(scaleFactor);
  rectMode(CENTER);
  strokeWeight(5 / scaleFactor);

  push();
  stroke(255);
  noFill();
  rect(0, 0, boxSize, boxSize);
  pop();

  // Draw the target point in blue if it exists
  if (motion_state?.target_pos) {
    const end = motion_state.target_pos;
    stroke(0, 0, 255);
    point(end[0], end[1]);
  }

  // Draw the path lines in white
  for (let i = 1; i < path.length; i++) {
    let a = path[i - 1];
    let b = path[i];
    stroke(255, 255, 255);
    line(a[0], a[1], b[0], b[1]);
  }

  // Draw the last point in red
  if (path.length > 0) {
    let last = path[path.length - 1];
    stroke(255, 0, 0);
    point(last[0], last[1]);
  }

  // Draw the first point in green (if more than one point exists)
  if (path.length > 1) {
    let first = path[0];
    stroke(0, 255, 0);
    point(first[0], first[1]);
  }
}

function onMotion(motion) {
  console.log(motion);

  motion_state = motion;
  path.push(motion.current_pos);
}
