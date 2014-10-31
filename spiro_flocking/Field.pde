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
      forces[i] = new PVector( random( -1, 1 ), 1.0 );
      forces[i].normalize();
    }
    radius = 200;
  }
  
  void render() {
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
  
  
  PVector force_for_location( PVector p ) {
    int xi = floor( p.x / block_w );
    int yi = floor( p.y / block_h );
    int loc = yi * w + xi;
    if (loc < 0 || loc >= count) {
      return new PVector( 0, 0 );
    }
    return forces[loc];
  }
  
  void paint() {
    
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
          float strength = map( d, 0, radius, 0.01, 0 );
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
