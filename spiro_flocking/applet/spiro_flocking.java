import processing.core.*; 
import processing.xml.*; 

import java.applet.*; 
import java.awt.Dimension; 
import java.awt.Frame; 
import java.awt.event.MouseEvent; 
import java.awt.event.KeyEvent; 
import java.awt.event.FocusEvent; 
import java.awt.Image; 
import java.io.*; 
import java.net.*; 
import java.text.*; 
import java.util.*; 
import java.util.zip.*; 
import java.util.regex.*; 

public class spiro_flocking extends PApplet {

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


public void setup() {
  size( 1200, 700 );
  smooth();
  steering = new PVector(0.0f, 0.0f);
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


public void draw() {
  background( 0 );

  steering = new PVector(0.0f, 0.0f);
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


public void keyPressed() {
  for(int u = 0; u < wilds.size(); u++){
    WildAgent w = (WildAgent)wilds.get(u);
    w.henshin();
  }
}

public void mouseDragged() {
}

public float easeIn( float t, float v1, float v2 ) {
  return v1 + (t * t * t) * (v2 - v1);
}


public float easeInOut( float t, float v1, float v2 ) {
  if ((t * 2) < 1)
    return v1 + ((t * 2) * (t * 2) * (t * 2)) * (v2 - v1) / 2.0f;
  else
    return v1 + ((t * 2 - 2) * (t * 2 - 2) * (t * 2 - 2) + 2) * (v2 - v1) / 2.0f;
}



class Agent {
  PVector prevLocation;
  PVector location;
  PVector velocity;
  float mass;
  float max_force;
  float max_speed;
  float orientation;
  PVector acceleration;
  float max_turn_rate;
  boolean seek_state;
  int clr;
  boolean wander_state;
  float wander_distance;
  float wander_strength;
  float wander_theta;
  float wander_rate;
  float stopping_distance;
  boolean data_capture;
  boolean data_show;
  float[] data;
  int data_count;
  float avoid_max_distance;
  float closest_distance;
  PVector closest_intersect;
  boolean flocking_state;
  float flock_distance;
  float flock_angle;
  int id;
  float cohesion_weight;
  float separation_weight;
  float alignment_weight;
  boolean flow_state;
  int neighborCount;


  Agent( float x, float y ) {
    id = next_id;
    next_id++;
    prevLocation = new PVector( x, y );
    location = new PVector( x, y );
    velocity = new PVector( 0, 0 );
    mass = 1.0f;
    max_force = 0.1f;
    //    max_speed = 3.0;
    max_speed = random(2.0f, 4.0f);
    orientation = 0;
    acceleration = new PVector( 0, 0 );
    max_turn_rate = TWO_PI / 10.0f;
    seek_state = false;
    clr = color( 222, 222, 222 );
    wander_state = false;
    wander_distance = 40.0f;
    wander_strength = 30.0f;
    wander_theta = 0;
    wander_rate = 0.09f;
    stopping_distance = 50.0f;
    data_capture = false;
    data_show = false;
    data = new float[1000];
    data_count = 0;
    avoid_max_distance = 150.0f;
    flocking_state = false;
    flock_distance = 150.0f;
    flock_angle = PI - PI/4.0f;
    cohesion_weight = random(0.3f, 0.6f);
    separation_weight = random(0.2f, 0.4f);
    alignment_weight = random(.5f, 1);
    flow_state = false;
    neighborCount = 0;
  }


  public void process() {
    prevLocation.x = location.x;
    prevLocation.y = location.y;
    PVector avoid_force = avoid();
    acceleration.add( avoid_force );

    if (avoid_force.mag() < .05f) {


      if (flow_state) {
        PVector flow_force = flow();
        acceleration.add( flow_force );
      }

      if (flocking_state) {
        ArrayList neighbors = find_neighbors( agents );
        neighborCount = neighbors.size();
        PVector cohesion_force = cohesion( neighbors );
        cohesion_force.normalize();
        cohesion_force.mult( cohesion_weight );
        acceleration.add( cohesion_force );

        PVector separation_force = separation( neighbors );
        separation_force.normalize();
        separation_force.mult( separation_weight );
        acceleration.add( separation_force );

        PVector alignment_force = alignment( neighbors );
        alignment_force.normalize();
        alignment_force.mult( alignment_weight );
        acceleration.add( alignment_force );
      }

      if (seek_state) {
        PVector seek_force = arrive( target1.location );
        acceleration.add( seek_force );
      }

      //PVector path_force = follow_path( path1 );
      //acceleration.add( path_force );

      if (wander_state) {
        PVector wander_force = wander();
        acceleration.add( wander_force );
      }
    }


    acceleration.div( mass );
    acceleration.limit( max_force );

    velocity.add( acceleration );
    velocity.limit( max_speed );

    if (velocity.mag() > 0.01f) {
      orientation = velocity.heading2D();
    }

    location.add( velocity );
    wrap();

    acceleration.x = 0;
    acceleration.y = 0;
  }


  public PVector flow() {
    PVector steer = new PVector( 0, 0 );

    PVector future_location = velocity.get();
    future_location.mult( 15 );
    future_location.add( location );

    PVector force = field.force_for_location( future_location );
    steer = PVector.sub( force, velocity );

    return steer;
  }


  public PVector alignment( ArrayList neighbors ) {
    PVector steer = new PVector( 0, 0 );
    if (neighbors.size() == 0)
      return steer;


    for (int i = 0; i < neighbors.size(); i++) {
      Agent a = (Agent)neighbors.get(i);
      PVector dir = a.velocity.get();
      float d = location.dist( a.location );
      float t = map( d, 0, flock_distance, 1.0f, 0.0f );
      dir.mult( t );
      steer.add( dir );
    }
    steer.div( neighbors.size() );
    return steer;
  }


  public PVector separation( ArrayList neighbors ) {
    PVector steer = new PVector( 0, 0 );
    if (neighbors.size() == 0)
      return steer;


    for (int i = 0; i < neighbors.size(); i++) {
      Agent a = (Agent)neighbors.get(i);
      PVector sep = PVector.sub( location, a.location );
      float r = sep.mag();
      sep.normalize();
      sep.mult( 1.0f / (r * r) );
      for (int u = 0; u < wilds.size(); u++) {
        WildAgent w = (WildAgent)wilds.get(u);
        if (a == w) {//separates especially hard if one of its neighbors is one of the wildcard ones
          sep.mult( 200);
        }
      }

      steer.add( sep );
    }

    return steer;
  }


  public PVector cohesion( ArrayList neighbors ) {
    PVector steer = new PVector( 0, 0 );
    if (neighbors.size() == 0)
      return steer;

    PVector cv = new PVector( 0, 0 );
    for (int i = 0; i < neighbors.size(); i++) {
      Agent a = (Agent)neighbors.get(i);
      cv.add( a.location );
    }
    cv.div( neighbors.size() );

    //    stroke( 255 );
    //    fill( 255, 0, 0 );
    //    ellipse( cv.x, cv.y, 10, 10 );

    steer = seek( cv );

    return steer;
  }


  public ArrayList find_neighbors( ArrayList agentlist ) {
    ArrayList nearby = new ArrayList();

    float a1 = map( flock_angle, 0, PI, 0, -PI );
    float a2 = flock_angle;
    //    stroke( 200 );
    //    fill( 200, 200, 200, 50 );
    //    pushMatrix();
    //    translate( location.x, location.y );
    //    rotate( orientation );
    //    arc( 0, 0, flock_distance * 2, flock_distance * 2, a1, a2 );
    //    popMatrix();

    PVector my_dir = velocity.get();
    my_dir.normalize();

    for (int i = 0; i < agentlist.size(); i++) {
      Agent a = (Agent)agentlist.get(i);
      if (id != a.id) {
        float d = location.dist( a.location );
        if (d < flock_distance) {
          PVector neighbor_dir = PVector.sub( a.location, location );
          neighbor_dir.normalize();
          float dot_product = my_dir.dot( neighbor_dir );
          float theta = acos( dot_product );
          if (theta < flock_angle) {
            a.clr = color(255, 105, 180 );
            nearby.add( a );
          }
          else {
            a.clr = color( 0, 255, 0 );
          }
        }
        else {
          a.clr = color( 128, 0, 128 );
        }
      }
    }

    return nearby;
  }


  public PVector avoid() {
    PVector steer = new PVector( 0, 0 );
    PVector direction = velocity.get();
    direction.normalize();
    direction.mult( avoid_max_distance );
    closest_distance = Float.MAX_VALUE;
    closest_intersect = new PVector( 0, 0 );
    boolean apply_steer = false;

    for (int i = 0; i < barriers.size(); i++) {
      Barrier b = (Barrier)barriers.get(i);
      PVector b_dir = PVector.sub( b.location, location );
      b_dir.normalize();
      PVector v_dir = velocity.get();
      v_dir.normalize();
      float dot_product = v_dir.dot( b_dir );
      if (dot_product > 0) {
        float distance_to_barrier = location.dist( b.location );
        if (distance_to_barrier < avoid_max_distance) {
          if (avoid_barrier( b, direction )) {
            PVector velocity_normal = velocity.get();
            velocity_normal.normalize();
            steer = new PVector( -velocity_normal.y, velocity_normal.x );
            PVector q = PVector.sub( b.location, location );
            float w = q.dot( steer );
            if (w > 0) {
              steer.mult( -1.0f );
            }
            float m = map( distance_to_barrier, 0, avoid_max_distance, max_speed, 0 );
            m = constrain( m, 0, max_speed );
            steer.mult( m );
            apply_steer = true;
          }
        }
      }
    }

    if (apply_steer) {
      //      stroke( 0, 255, 0 );
      //      strokeWeight( 2 );
      //      line( location.x, location.y, closest_intersect.x, closest_intersect.y );

      //stroke( 255, 0, 0 );
      //line( location.x, location.y, location.x + steer.x * 10.0, location.y + steer.y * 10.0 );
    }

    return steer;
  }


  public boolean avoid_barrier( Barrier b, PVector direction ) {
    PVector a0 = new PVector( location.x - b.location.x, location.y - b.location.y );
    PVector b0 = new PVector( location.x + direction.x - b.location.x, location.y + direction.y - b.location.y );
    float dx = b0.x - a0.x;
    float dy = b0.y - a0.y;
    float dr = sqrt( dx * dx + dy * dy );
    float d  = a0.x * b0.y - b0.x * a0.y;
    float discrim = ((b.radius + 5) * (b.radius + 5) * dr * dr) - (d * d);
    if (discrim > 0) {
      float sqrt_discrim = sqrt( discrim );
      float x1 = (d * dy + sgn(dy) * dx * sqrt_discrim) / (dr * dr);
      float y1 = (-d * dx + abs(dy) * sqrt_discrim) / (dr * dr);
      PVector p1 = new PVector( x1, y1 );
      float x2 = (d * dy - sgn(dy) * dx * sqrt_discrim) / (dr * dr);
      float y2 = (-d * dx - abs(dy) * sqrt_discrim) / (dr * dr);
      PVector p2 = new PVector( x2, y2 );
      float distance_p1 = a0.dist( p1 );
      float distance_p2 = a0.dist( p2 );
      float distance_to_barrier = min( distance_p1, distance_p2 );

      if (distance_to_barrier < closest_distance) {
        stroke( 255 );
        //line( location.x, location.y, b.location.x, b.location.y );
        closest_distance = distance_to_barrier;
        if (distance_p1 < distance_p2) {
          closest_intersect.x = b.location.x + x1;
          closest_intersect.y = b.location.y + y1;
        }
        else {
          closest_intersect.x = b.location.x + x2;
          closest_intersect.y = b.location.y + y2;
        }
        //line( location.x, location.y, closest_intersect.x, closest_intersect.y );
        return true;
      }
    }
    return false;
  }

  public float sgn( float x ) {
    if (x < 0)
      return -1.0f;
    else
      return 1.0f;
  }

  public PVector arrive( PVector target ) {
    PVector steer = seek( target );
    float distance_to_target = location.dist( target );
    if (distance_to_target < stopping_distance) {
      float t = map( distance_to_target, 0, stopping_distance, 0, 1 );
      //t = easeIn( t, 0, 1 );

      if (data_capture) {
        data[data_count] = t;
        data_count++;
      }

      steer.mult( t );
      steer.sub( velocity );
    }
    return steer;
  }

  public void render_data() {
    stroke( 160 );
    fill( 50 );
    float w = 400;
    float h = 400;
    float mw = (width - w) / 2.0f;
    float mh = (height - h) / 2.0f;
    rect( mw, mh, w, h );
    stroke( 255 );
    float x = mw;
    float xinc = w / (float)data_count;
    for (int i = 1; i < data_count; i++) {

      float y1 = map( data[i-1], 0.0f, 1.0f, h + mh, mh );
      float y2 = map( data[i], 0.0f, 1.0f, h + mh, mh );
      line( x, y1, x + xinc, y2 );
      x += xinc;
    }
  }


  public PVector wander() {
    PVector direction = new PVector( cos( orientation ), sin( orientation ) );
    direction.normalize();
    direction.mult( wander_distance );
    PVector wander_center = PVector.add( location, direction );

    float wx = wander_center.x + wander_strength * cos( wander_theta );
    float wy = wander_center.y + wander_strength * sin( wander_theta );

    //stroke(255);
    //strokeWeight(1);
    //ellipse( wander_center.x, wander_center.y, wander_strength * 2, wander_strength * 2 );
    //ellipse( wx, wy, 10, 10 );

    wander_theta += ImprovedNoise.noise(location.x/100.0f+frameCount/60.0f, location.y/100.0f, 0) * wander_rate;

    PVector steer = new PVector( wx - location.x, wy - location.y );
    return steer;
  }


  public PVector follow_path( Path path ) {
    PVector steer = new PVector( 0.0f, 0.0f );

    float m = map( velocity.mag(), 0.5f, 5, 20, 50 );
    m = constrain( m, 20, 50 );
    PVector predicted_location = PVector.add( location, PVector.mult( velocity, m ) );

    PVector p = path.points.get( 0 );
    PVector b = new PVector( 0, 0 );
    PVector closest = new PVector( 0, 0 );
    float closest_distance = Float.MAX_VALUE;
    for (int i = 0; i < path.points.size() - 1; i++) {
      PVector q = path.points.get( i + 1 );

      float t = closest_point_on_line( predicted_location, p, q, b );
      float d = predicted_location.dist( b );
      if (d < closest_distance) {
        closest_distance = d;
        closest.x = b.x;
        closest.y = b.y;
      }

      p = q;
    }

    if (closest_distance > path.radius) {
      steer = seek( closest );
    }
    return steer;
  }

  public float closest_point_on_line( PVector a, PVector p1, PVector p2, PVector b ) {
    // vector from p1 to p2
    PVector line_seg = PVector.sub( p1, p2 );
    // vector perpendicular to the line
    PVector line_perp = new PVector( line_seg.y, -line_seg.x );

    // vector r is a vector from a to the line
    PVector r = PVector.sub ( p1, a );

    // project r onto line_perp
    float d1 = line_seg.x * r.y - r.x * line_seg.y;
    float d = abs(d1) / line_seg.mag();

    // if d1 is positive, reverse d, so that line_perp flips 
    // to point towards the line.
    if (d1 > 0)
      d = d * -1;

    // make line_perp vector be length d
    line_perp.normalize();
    line_perp.mult( d );

    // b is the new point, on the line, closest to the point a.
    // however, this point may not be on the *line segment*
    b.x = a.x + line_perp.x;
    b.y = a.y + line_perp.y;

    // t is where b is parametericly on this line.
    // if t is between 0-1, then it is on the line segment.
    float t = (b.x / (p2.x - p1.x)) - (p1.x / (p2.x - p1.x));

    if (t < 0) {
      // if t < 0, then b was on the line before p1
      b.x = p1.x;
      b.y = p1.y;
    }
    else if (t > 1) {
      // if t > 1, then b was on the line after p2
      b.x = p2.x;
      b.y = p2.y;
    }

    return t;
  }


  public PVector seek( PVector seek_position ) {
    PVector desired_velocity = PVector.sub( seek_position, location );
    desired_velocity.normalize();
    float desired_heading = desired_velocity.heading2D();
    float heading_diff = desired_heading - orientation;
    if (heading_diff > PI) {
      heading_diff = -(TWO_PI - heading_diff);
    }
    else if (heading_diff < -PI) {
      heading_diff = TWO_PI - abs( heading_diff );
    }

    //    noStroke();
    //    fill( 255 );
    //    text( heading_diff, 10, 50 );

    float turn_delta = constrain( heading_diff, -max_turn_rate, max_turn_rate );
    float desire = orientation + turn_delta;
    PVector seek = new PVector( cos( desire ) * max_speed, sin( desire ) * max_speed );
    return seek;
    //    desired_velocity.mult( max_speed );
    //    steering_direction = PVector.sub( desired_velocity, velocity );
  }


  public PVector pursuit( Agent target ) {
    PVector predicted_velocity = target.velocity.get();
    float dot_product = velocity.dot( target.velocity );

    float t = 1.0f;
    if (dot_product < 0) {
      t = 50.0f;
    }
    else {
      float d = location.dist( target.location );
      t = map( d, 10.0f, 200.0f, 1.0f, 50.0f );
      t = constrain( t, 1, 50 );
    }
    predicted_velocity.mult( t );
    PVector predicted = PVector.add( target.location, predicted_velocity );
    PVector seek = seek( predicted );
    return seek;
  }


  public void thrust( float magnitude ) {
    float vx = magnitude * cos( orientation );
    float vy = magnitude * sin( orientation );
    velocity.x += vx;
    velocity.y += vy;
  }

  public void turn( float magnitude ) {
    acceleration.x = velocity.y;
    acceleration.y = -velocity.x;
    acceleration.normalize();
    acceleration.mult( magnitude );
  }


  public void wrap() {
    if (location.x > width)
      location.x = 0;
    else if (location.x < 0)
      location.x = width;
    if (location.y > height) 
      location.y = 0;
    else if (location.y < 0)
      location.y = height;
  }


  public void render() {
    pushMatrix();

    translate( location.x, location.y );
    scale(((float)neighborCount * 10/(float)agents.size()));// Scale to percent of agents that are its neighbors
    rotate( orientation );
    stroke( clr );
    strokeWeight( 1 );
    fill( 0 );
    triangle( -6, -5, -6, 5, 6, 0 );
    popMatrix();

    if (data_show) {
      render_data();
    }
  }
}


float arrow__size = 5.0f;

public void arrowSize( float newArrowSize ) {
  arrow__size = newArrowSize;
}

public void arrow( float x1, float y1, float x2, float y2 ) {
  
  line( x1, y1, x2, y2 );
  
  PVector v = new PVector( x2 - x1, y2 - y1 );
  float theta = v.heading2D();
  
  pushMatrix();
  translate( x2, y2 );
  rotate( theta );
  line( -arrow__size, -arrow__size, 0, 0 );
  line(  0,  0, -arrow__size, arrow__size );
  popMatrix();
  
}
class Barrier {
  PVector location;
  float radius;
  
  Barrier( float x, float y, float r ) {
    location = new PVector( x, y );
    radius = r;
  }
  
  public void render() {
    stroke( 0 );
    strokeWeight( 1 );
    fill( 255 );
    ellipse( location.x, location.y, radius * 2, radius * 2 );
  }
  
}
class Field {
  int block_w;
  int block_h;
  int w;
  int h;
  int count;
  PVector[] forces;
  float radius;
  
  Field() {
    block_w = 100;
    block_h = 100;
    w = width / block_w;
    h = height / block_h;
    count = w * h;
    forces = new PVector[count];
    for (int i = 0; i < count; i++) {
      forces[i] = new PVector( random( -1, 1 ), 1.0f );
      forces[i].normalize();
    }
    radius = 200;
  }
  
  public void render() {
    noFill();
    stroke( 10, 230, 10 );
    float x;
    float y = block_h / 2;
    int k = 0;
    for (int i = 0; i < h; i++) {
      x = block_w / 2;
      for (int j = 0; j < w; j++) {
        arrow( x, y, x + forces[k].x * 20, y + forces[k].y * 20 );
        x += block_w;
        k++;
      }
      y += block_h;
    }
  }
  
  
  public PVector force_for_location( PVector p ) {
    int xi = floor( p.x / block_w );
    int yi = floor( p.y / block_h );
    int loc = yi * w + xi;
    if (loc < 0 || loc >= count) {
      return new PVector( 0, 0 );
    }
    return forces[loc];
  }
  
  public void paint() {
    
    stroke( 0, 255, 0 );
    noFill();
    ellipse( mouseX, mouseY, radius * 2, radius * 2 );
    
    float x;
    float y = block_h / 2;
    int k = 0;
    for (int i = 0; i < h; i++) {
      x = block_w / 2;
      for (int j = 0; j < w; j++) {
        float d = dist( mouseX, mouseY, x, y );
        if (d < radius) {
          float strength = map( d, 0, radius, 0.01f, 0 );
          PVector v = PVector.sub( new PVector( mouseX, mouseY ), new PVector( x, y ) );
          v.mult( strength );
          forces[k].add( v );
          forces[k].normalize();
        }
        
        x += block_w;
        k++;
      }
      y += block_h;
    }
  }
  
}
class FlockAgent extends Agent {

  FlockAgent(float x, float y) {
    super(x, y);
    flocking_state = true;
  }
  
}
class Path {
  ArrayList<PVector> points;
  float radius;
  
  Path() {
    radius = 20.0f;
    points = new ArrayList<PVector>();
  }
  
  
  public void addPoint( float x, float y ) {
    PVector p = new PVector( x, y );
    points.add( p );
  }
  
  
  public void render() {
    stroke( 50 );
    strokeWeight( radius * 2 );
    noFill();
    beginShape();
    for (PVector p : points) {
      vertex( p.x, p.y );
    }
    endShape();
    
    stroke( 150 );
    strokeWeight( 2 );
    noFill();
    beginShape();
    for (PVector p : points) {
      vertex( p.x, p.y );
    }
    endShape();
  }
  
}
class Target {
  PVector location;
  
  Target( float x, float y ) {
    location = new PVector( x, y );
  }
  
  
  public void render() {
    stroke( 0, 200, 0 );
    strokeWeight( 2 );
    fill( 0 );
    pushMatrix();
    translate( location.x, location.y );
    rotate( PI / 4 );
    ellipse( 0, 0, 16, 16 );
    line( -8, 0, 8, 0 );
    line( 0, -8, 0, 8 );
    popMatrix();
  }
  
}
class WildAgent extends Agent {
  int arrTarInd;// Arrival target index (where in the arraylist they are on their targets
  int state;
  boolean app_state;
  Agent targ;
  

  WildAgent(float x, float y) {
    super(x, y);
    flocking_state = false;
    wander_state = false;
    seek_state = false;
    app_state = false;
    arrTarInd = 0;
    state = 0;
    max_speed = random(4.0f, 8.0f);

  }

  public void process() {
    prevLocation.x = location.x;
    prevLocation.y = location.y;
    PVector avoid_force = avoid();
    acceleration.add( avoid_force );

    if (avoid_force.mag() < .05f || seek_state ) {


      if (flocking_state) {
        ArrayList neighbors = find_neighbors( agents );
        neighborCount = neighbors.size();
        PVector cohesion_force = cohesion( neighbors );
        cohesion_force.normalize();
        cohesion_force.mult( cohesion_weight );
        acceleration.add( cohesion_force );

        PVector separation_force = separation( neighbors );
        separation_force.normalize();
        separation_force.mult( separation_weight );
        acceleration.add( separation_force );

        PVector alignment_force = alignment( neighbors );
        alignment_force.normalize();
        alignment_force.mult( alignment_weight );
        acceleration.add( alignment_force );
      }

      if (seek_state) {
        Barrier beta = (Barrier) barriers.get(arrTarInd);
        PVector bLand = beta.location;
        stroke(0,0,0,128);
        fill(0,0,0,128);
        ellipse(bLand.x, bLand.y, 40, 40);
        PVector seek_force = arrive( bLand);
        acceleration.add( seek_force );
        float distB = dist(location.x, location.y, bLand.x, bLand.y);
        if (distB<100) {
          acceleration.sub( avoid_force );//to cancel out avoiding so that the approach can actually happen
          acceleration.sub(PVector.mult(seek_force, (100- distB)));
          if (distB<70) {
            if (arrTarInd < barriers.size()-1) {
              arrTarInd++;
            }
            else {
              arrTarInd = 0;
            }
          }
        }
      }

      if (app_state) {
          PVector run_force = new PVector(0,0);
          targ = (Agent)agents.get(0);
          ellipse(targ.location.x, targ.location.y, 30, 30);
          run_force = pursuit( targ);
          acceleration.add( PVector.mult(run_force, 1) );
      }

      if (wander_state) {
        PVector wander_force = wander();
        acceleration.add( wander_force );
      }
    }


    acceleration.div( mass );
    acceleration.limit( max_force );

    velocity.add( acceleration );
    velocity.limit( max_speed );

    if (velocity.mag() > 0.01f) {
      orientation = velocity.heading2D();
    }

    location.add( velocity );
    wrap();

    acceleration.x = 0;
    acceleration.y = 0;
  }

  public void render() {
    pushMatrix();
    translate( location.x, location.y );
    //scale(((float)neighborCount * 10/(float)agents.size()));// Scale to percent of agents that are its neighbors
    scale(5);
    rotate( orientation );
    clr = color(0, 255, 0);
    stroke( clr );
    strokeWeight( 1 );
    fill( 0 );
    triangle( -6, -5, -6, 5, 6, 0 );
    stroke(255);
    beginShape();
    vertex(6, 0);
    vertex(4, -2);
    vertex(-4, -2);
    vertex(-4, 2);
    vertex(4, 2);
    endShape(CLOSE);
    pushMatrix();
    float percento = 300/velocity.mag();
    //float flap = sin(TWO_PI*((float)frameCount%30)/30);
    float flap = sin(TWO_PI*((float)frameCount%percento)/percento);
    scale(1, flap);
    stroke(255, 0, 0);
    beginShape();
    vertex(-3, -3);
    vertex(-5, -6);
    vertex(3, -3);
    vertex(1, -1);
    vertex(-3, 4);
    vertex(-5, 6);
    vertex(3, 4);
    vertex(-1, -1);
    endShape(CLOSE);
    popMatrix();
    stroke(255, 255, 0);
    triangle(3, -.5f, 3, .5f, 5, 0 );
    popMatrix();

    if (data_show) {
      render_data();
    }
  }

  public PVector wander() {
    PVector direction = new PVector( cos( orientation ), sin( orientation ) );
    direction.normalize();
    direction.mult( wander_distance );
    PVector wander_center = PVector.add( location, direction );

    float wx = wander_center.x + wander_strength * cos( wander_theta );
    float wy = wander_center.y + wander_strength * sin( wander_theta );

    stroke(255);
    strokeWeight(1);
//    ellipse( wander_center.x, wander_center.y, wander_strength * 2, wander_strength * 2 );
    ellipse( wx, wy, 10, 10 );

    wander_theta += ImprovedNoise.noise(location.x/100.0f+frameCount/60.0f, location.y/100.0f, 0) * wander_rate;

    PVector steer = new PVector( wx - location.x, wy - location.y );
    return steer;
  }

  public void henshin() {
    if (state < 3) {
      state++;
    }
    else {
      state = 0;
    }
    switch (state) {
    case 0:  
      app_state = false;
      wander_state = false;
      seek_state = false;
      break;
    case 1:
      app_state = true;
      wander_state = false;
      seek_state = false;
      break;
    case 2:  
      app_state = false;
      wander_state = true;
      seek_state = false;
      break;
    case 3:  
      app_state = false;
      wander_state = false;
      seek_state = true;
      break;
    }
    println(state);
  }

  public PVector pursuit( FlockAgent target ) {
    PVector predicted_velocity = target.velocity.get();// puts the targets velocity into a variable called predicted velocity
    predicted_velocity.mult( 60 );
    PVector predicted = PVector.add( target.location, predicted_velocity );
    PVector arrival = arrival( predicted, 20 );
    return arrival;
  }
  public PVector arrival( PVector arrival_position, float proximity ) {
    PVector target_offset = PVector.sub(arrival_position, location);
    float thisDistance = target_offset.mag();
    float ramped_speed = max_speed * ((thisDistance- proximity)/ 100 ); /* Magic number but should be an area around the target*/
    if ((ramped_speed < 2)&& (thisDistance < proximity)) {
      ramped_speed = 0;
    }
    float clipped_speed = min(ramped_speed, max_speed);// Constrains true speed to below max
    PVector desired_velocity = PVector.mult(target_offset, (clipped_speed / thisDistance));
    PVector steering = PVector.sub(desired_velocity, velocity);
    return steering;
  }
}

  static public void main(String args[]) {
    PApplet.main(new String[] { "--bgcolor=#F0F0F0", "spiro_flocking" });
  }
}
