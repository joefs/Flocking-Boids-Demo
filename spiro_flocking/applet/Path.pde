class Path {
  ArrayList<PVector> points;
  float radius;
  
  Path() {
    radius = 20.0;
    points = new ArrayList<PVector>();
  }
  
  
  void addPoint( float x, float y ) {
    PVector p = new PVector( x, y );
    points.add( p );
  }
  
  
  void render() {
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
