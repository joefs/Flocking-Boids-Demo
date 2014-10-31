
float arrow__size = 5.0;

void arrowSize( float newArrowSize ) {
  arrow__size = newArrowSize;
}

void arrow( float x1, float y1, float x2, float y2 ) {
  
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
