let width = 300;
let height = 300;

function setup() {
  createCanvas(width, height);
}

let motion_state = null;

let path = [];

function draw() {
  background(0, 0, 0);

  translate(width / 2, height / 2);

  if (motion_state?.target_pos) {
    const end = motion_state.target_pos;
    stroke(0, 0, 255);
    strokeWeight(4);
    point(end[0], end[1]);
  }

  for (let i = 1; i < path.length; i++) {
    let a = path[i - 1];
    let b = path[i];

    stroke(255, 255, 255);
    strokeWeight(2);
    line(a[0], a[1], b[0], b[1]);
  }

  if (path.length > 0) {
    let last = path[path.length - 1];

    stroke(255, 0, 0);
    strokeWeight(4);
    point(last[0], last[1]);
  }

  if (path.length > 1) {
    let first = path[0];

    stroke(0, 255, 0);
    strokeWeight(4);
    point(first[0], first[1]);
  }
}

function onMotion(motion) {
  console.log(motion);

  motion_state = motion;
  path.push(motion.current_pos);
}
