class Barrier {
  PVector location;
  float radius;
  
  Barrier( float x, float y, float r ) {
    location = new PVector( x, y );
    radius = r;
  }
  
  void render() {
    stroke( 0 );
    strokeWeight( 1 );
    fill( 255 );
    ellipse( location.x, location.y, radius * 2, radius * 2 );
  }
  
}
