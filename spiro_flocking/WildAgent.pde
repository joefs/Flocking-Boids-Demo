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
    max_speed = random(4.0, 8.0);

  }

  void process() {
    prevLocation.x = location.x;
    prevLocation.y = location.y;
    PVector avoid_force = avoid();
    acceleration.add( avoid_force );

    if (avoid_force.mag() < .05 || seek_state ) {


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

    if (velocity.mag() > 0.01) {
      orientation = velocity.heading2D();
    }

    location.add( velocity );
    wrap();

    acceleration.x = 0;
    acceleration.y = 0;
  }

  void render() {
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
    triangle(3, -.5, 3, .5, 5, 0 );
    popMatrix();

    if (data_show) {
      render_data();
    }
  }

  PVector wander() {
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

    wander_theta += ImprovedNoise.noise(location.x/100.0+frameCount/60.0, location.y/100.0, 0) * wander_rate;

    PVector steer = new PVector( wx - location.x, wy - location.y );
    return steer;
  }

  void henshin() {
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

  PVector pursuit( FlockAgent target ) {
    PVector predicted_velocity = target.velocity.get();// puts the targets velocity into a variable called predicted velocity
    predicted_velocity.mult( 60 );
    PVector predicted = PVector.add( target.location, predicted_velocity );
    PVector arrival = arrival( predicted, 20 );
    return arrival;
  }
  PVector arrival( PVector arrival_position, float proximity ) {
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

