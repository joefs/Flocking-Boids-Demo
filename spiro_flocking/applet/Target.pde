class Target {
  PVector location;
  
  Target( float x, float y ) {
    location = new PVector( x, y );
  }
  
  
  void render() {
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
