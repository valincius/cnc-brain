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

let scaleFactor = 5;

let path = [];

function draw() {
  background(0, 0, 0);

  translate(width / 2, height / 2);
  scale(scaleFactor);

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
  console.log('motion', motion);

  path.push(motion.current_pos);
}