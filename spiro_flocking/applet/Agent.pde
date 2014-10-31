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
  color clr;
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
    mass = 1.0;
    max_force = 0.1;
    //    max_speed = 3.0;
    max_speed = random(2.0, 4.0);
    orientation = 0;
    acceleration = new PVector( 0, 0 );
    max_turn_rate = TWO_PI / 10.0;
    seek_state = false;
    clr = color( 222, 222, 222 );
    wander_state = false;
    wander_distance = 40.0;
    wander_strength = 30.0;
    wander_theta = 0;
    wander_rate = 0.09;
    stopping_distance = 50.0;
    data_capture = false;
    data_show = false;
    data = new float[1000];
    data_count = 0;
    avoid_max_distance = 150.0;
    flocking_state = false;
    flock_distance = 150.0;
    flock_angle = PI - PI/4.0;
    cohesion_weight = random(0.3, 0.6);
    separation_weight = random(0.2, 0.4);
    alignment_weight = random(.5, 1);
    flow_state = false;
    neighborCount = 0;
  }


  void process() {
    prevLocation.x = location.x;
    prevLocation.y = location.y;
    PVector avoid_force = avoid();
    acceleration.add( avoid_force );

    if (avoid_force.mag() < .05) {


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

    if (velocity.mag() > 0.01) {
      orientation = velocity.heading2D();
    }

    location.add( velocity );
    wrap();

    acceleration.x = 0;
    acceleration.y = 0;
  }


  PVector flow() {
    PVector steer = new PVector( 0, 0 );

    PVector future_location = velocity.get();
    future_location.mult( 15 );
    future_location.add( location );

    PVector force = field.force_for_location( future_location );
    steer = PVector.sub( force, velocity );

    return steer;
  }


  PVector alignment( ArrayList neighbors ) {
    PVector steer = new PVector( 0, 0 );
    if (neighbors.size() == 0)
      return steer;


    for (int i = 0; i < neighbors.size(); i++) {
      Agent a = (Agent)neighbors.get(i);
      PVector dir = a.velocity.get();
      float d = location.dist( a.location );
      float t = map( d, 0, flock_distance, 1.0, 0.0 );
      dir.mult( t );
      steer.add( dir );
    }
    steer.div( neighbors.size() );
    return steer;
  }


  PVector separation( ArrayList neighbors ) {
    PVector steer = new PVector( 0, 0 );
    if (neighbors.size() == 0)
      return steer;


    for (int i = 0; i < neighbors.size(); i++) {
      Agent a = (Agent)neighbors.get(i);
      PVector sep = PVector.sub( location, a.location );
      float r = sep.mag();
      sep.normalize();
      sep.mult( 1.0 / (r * r) );
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


  PVector cohesion( ArrayList neighbors ) {
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


  ArrayList find_neighbors( ArrayList agentlist ) {
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


  PVector avoid() {
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
              steer.mult( -1.0 );
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


  boolean avoid_barrier( Barrier b, PVector direction ) {
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

  float sgn( float x ) {
    if (x < 0)
      return -1.0;
    else
      return 1.0;
  }

  PVector arrive( PVector target ) {
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

  void render_data() {
    stroke( 160 );
    fill( 50 );
    float w = 400;
    float h = 400;
    float mw = (width - w) / 2.0;
    float mh = (height - h) / 2.0;
    rect( mw, mh, w, h );
    stroke( 255 );
    float x = mw;
    float xinc = w / (float)data_count;
    for (int i = 1; i < data_count; i++) {

      float y1 = map( data[i-1], 0.0, 1.0, h + mh, mh );
      float y2 = map( data[i], 0.0, 1.0, h + mh, mh );
      line( x, y1, x + xinc, y2 );
      x += xinc;
    }
  }


  PVector wander() {
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

    wander_theta += ImprovedNoise.noise(location.x/100.0+frameCount/60.0, location.y/100.0, 0) * wander_rate;

    PVector steer = new PVector( wx - location.x, wy - location.y );
    return steer;
  }


  PVector follow_path( Path path ) {
    PVector steer = new PVector( 0.0, 0.0 );

    float m = map( velocity.mag(), 0.5, 5, 20, 50 );
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

  float closest_point_on_line( PVector a, PVector p1, PVector p2, PVector b ) {
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


  PVector seek( PVector seek_position ) {
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


  PVector pursuit( Agent target ) {
    PVector predicted_velocity = target.velocity.get();
    float dot_product = velocity.dot( target.velocity );

    float t = 1.0;
    if (dot_product < 0) {
      t = 50.0;
    }
    else {
      float d = location.dist( target.location );
      t = map( d, 10.0, 200.0, 1.0, 50.0 );
      t = constrain( t, 1, 50 );
    }
    predicted_velocity.mult( t );
    PVector predicted = PVector.add( target.location, predicted_velocity );
    PVector seek = seek( predicted );
    return seek;
  }


  void thrust( float magnitude ) {
    float vx = magnitude * cos( orientation );
    float vy = magnitude * sin( orientation );
    velocity.x += vx;
    velocity.y += vy;
  }

  void turn( float magnitude ) {
    acceleration.x = velocity.y;
    acceleration.y = -velocity.x;
    acceleration.normalize();
    acceleration.mult( magnitude );
  }


  void wrap() {
    if (location.x > width)
      location.x = 0;
    else if (location.x < 0)
      location.x = width;
    if (location.y > height) 
      location.y = 0;
    else if (location.y < 0)
      location.y = height;
  }


  void render() {
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

