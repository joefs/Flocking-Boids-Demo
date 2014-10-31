ArrayList agents;
Agent target1;
PFont f;
Path path1;
ArrayList barriers;
int next_id;
Field field;
ArrayList neighbors;
PVector steering;
ArrayList wilds;


void setup() {
  size( 1200, 700 );
  smooth();
  steering = new PVector(0.0, 0.0);
  field = new Field();
  next_id = 0;

  f = createFont( "Helvetica", 24 );
  textFont( f );


  agents = new ArrayList();
  for (int i = 0; i < 77; i++) {
    Agent f = new FlockAgent( random(width), random(height) );
    f.velocity.x = random(-3, 3);
    f.velocity.y = random(-3, 3);
    f.flocking_state = true;
    agents.add( f );
  }
  //
  wilds = new ArrayList();
  //
  WildAgent w = new WildAgent( random(width), random(height) );
  w.velocity.x = random(-3, 3);
  w.velocity.y = random(-3, 3);
  agents.add( w );
  wilds.add( w );
  w = new WildAgent( random(width), random(height) );
  w.velocity.x = random(-3, 3);
  w.velocity.y = random(-3, 3);
  agents.add( w );
  wilds.add( w );
  //
  barriers = new ArrayList();
  Barrier b1 = new Barrier( 300, 300, 60 );
  barriers.add( b1 );
  b1 = new Barrier( 800, 500, 60 );
  barriers.add( b1 );
  b1 = new Barrier( 950, 200, 60 );
  barriers.add( b1 );
  b1 = new Barrier( 100, 100, 60 );
  barriers.add( b1 );
  b1 = new Barrier( width/2, height/2 -150, 60 );
  barriers.add( b1 );
}


void draw() {
  background( 0 );

  steering = new PVector(0.0, 0.0);
  neighbors = new ArrayList();
  boolean avoiding;
  for (int i = 0; i < barriers.size(); i++) {
    Barrier b = (Barrier)barriers.get(i);

    b.render();
  }

  for (int i = 0; i < agents.size(); i++) {

    Agent a = (Agent)agents.get(i);
    a.process();
    a.render();
  }
  fill(255);
  text("0 = Neutral, 1 = give chase 2 = wander 3 = dive",width/2-225, height-35);
  text("press any key to toggle the wildcard states",width/2-200, height-15);
}


void keyPressed() {
  for(int u = 0; u < wilds.size(); u++){
    WildAgent w = (WildAgent)wilds.get(u);
    w.henshin();
  }
}

void mouseDragged() {
}

float easeIn( float t, float v1, float v2 ) {
  return v1 + (t * t * t) * (v2 - v1);
}


float easeInOut( float t, float v1, float v2 ) {
  if ((t * 2) < 1)
    return v1 + ((t * 2) * (t * 2) * (t * 2)) * (v2 - v1) / 2.0;
  else
    return v1 + ((t * 2 - 2) * (t * 2 - 2) * (t * 2 - 2) + 2) * (v2 - v1) / 2.0;
}



