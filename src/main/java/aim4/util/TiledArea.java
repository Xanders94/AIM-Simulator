/*
Copyright (c) 2011 Tsz-Chiu Au, Peter Stone
University of Texas at Austin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Texas at Austin nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package aim4.util;

import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import aim4.voronoi.GraphEdge;
import aim4.voronoi.Vertex;
import aim4.voronoi.Voronoi;

/**
 * A tiled area - a subdivision of an area into a grid of small rectangles.
 */
public class TiledArea {

  /////////////////////////////////
  // NESTED CLASSES
  /////////////////////////////////

  /**
   * A tile.
   */
	public static interface Tile {
		public int getX();
		
		public int getY();
		
		public int getId();
		
		public boolean isEdgeTile();
		
		public void setEdgeTile(boolean edgeTile);
		
		public Path2D getShape();
	}
	
  public static class SquareTile implements Tile {
    /** The area controlled by this tile. */
    private final Rectangle2D rectangle;
    /** the x-coordinate of this tile */
    private final int x;
    /** the y-coordinate of this tile */
    private final int y;
    /** the id of this tile */
    private final int id;
    /** whether or not a tile is on the edge */
    private boolean edgeTile = false;

    /**
     * Create a tile.
     *
     * @param rectangle  the area of the tile
     * @param x          the x-coordinate of the tile
     * @param y          the y-coordinate of the tile
     * @param id         the ID of the tile
     */
    public SquareTile(Rectangle2D rectangle, int x, int y, int id) {
      this.rectangle = rectangle;
      this.x = x;
      this.y = y;
      this.id = id;
    }

    /** Get the area controlled by this ReservationTile. 
     * @return rectangle as a Rectangle2D object
     */
    public Rectangle2D getRectangle() {
      return rectangle;
    }

    /** Get the x-coordinate of this tile */
    public int getX() {
      return x;
    }

    /** Get the y-coordinate of this tile */
    public int getY() {
      return y;
    }

    /** Get the id of this tile */
    public int getId() {
      return id;
    }

    /** Whether or not this tile is on the edge */
    public boolean isEdgeTile() {
      return edgeTile;
    }

    /**
     * Set whether or not this tile is on the edge.
     *
     * @param edgeTile  whether or not this tile is on the edge
     */
    public void setEdgeTile(boolean edgeTile) {
      this.edgeTile = edgeTile;
    }

	@Override
	public Path2D getShape() {
		return new Path2D.Double(rectangle);
	}
  }

  /////////////////////////////////
  // PRIVATE FIELDS
  /////////////////////////////////

  /** The area controlled by this tiled area. */
  private final Area area;
  /** The bounding rectangle controlled by this tiled area. */
  private final Rectangle2D rectangle;
  /** The number of tiles in the x-direction */
  private final int xNum;
  /** The number of tiles in the y-direction. */
  private final int yNum;
  /** The length of the tiles in the x-direction. */
  private final double xLength;
  /** The length of the tiles in the y-direction. */
  private final double yLength;
  /** The tiles in this area. */
  private Tile[][] tiles;
  /** A mapping from id to tiles */
  private ArrayList<Tile> idToTiles;
  /** The number of tiles */
  private int numberOfTiles;
  /** an indication that the tiles are square*/
  private boolean rectangualarTiles;
  /** stores different configurations of tiles*/
  private ArrayList<ArrayList<Tile>> idToTilesStore = new ArrayList<ArrayList<Tile>>();
  private ArrayList<Tile[][]> tilesStore = new ArrayList<Tile[][]>();
  private int tileSetId;

  /////////////////////////////////
  // CLASS CONSTRUCTORS
  /////////////////////////////////

  /**
   * Create a tiled area
   *
   * @param area    the area
   * @param length  the length of a tile in both directions
   */
  public TiledArea(Area area, double length) {
    this(area, length, length);
    this.rectangualarTiles = true;
  }

  /**
   * Create a tiled area
   *
   * @param area     the area
   * @param xLength  the length of a tile in the x-direction
   * @param yLength  the length of a tile in the y-direction
   */
  public TiledArea(Area area, double xLength, double yLength) {
    this.area = area;
    this.rectangle = area.getBounds2D();
    this.xLength = xLength;
    this.yLength = yLength;
    this.rectangualarTiles = true;
    xNum = ((int)(rectangle.getWidth() / xLength)) + 1;
    yNum = ((int)(rectangle.getHeight() / yLength)) + 1;
    tiles = new SquareTile[xNum][yNum];
    idToTiles = new ArrayList<Tile>(xNum*yNum) ;
    createTiles();
    identifyEdgeTiles();
  }
  public TiledArea(Area area, List<Point2D> points,int padding){
	  this.area = area;
	  this.rectangle = area.getBounds2D();
	  this.xLength = rectangle.getWidth()/(points.size() + 1);
	  this.yLength = rectangle.getHeight()/(points.size() + 1);
	  this.rectangualarTiles = false;
	  xNum = ((int)(rectangle.getWidth() / xLength)) + 1;
	  yNum = ((int)(rectangle.getHeight() / yLength)) + 1;
	  tiles = new PolyTile[xNum][yNum];
	  idToTiles = new ArrayList<Tile>(xNum*yNum);
	  tileSetId = 0;
	  createPolyTiles(points, padding);
	  identifyEdgeTiles();
  }

  /**
   * Create the tiles
   */
  private void createTiles() {
	  numberOfTiles = 0;
      for(int x = 0; x < xNum; x++) {
        for(int y = 0; y < yNum; y++) {
          // Create a tile
          // Start by finding the offset for this particular tile
          double xOffset = x * xLength;
          double yOffset = y * yLength;
          // These should be granularity most of the time, except on the
          // last row/column
          double width = Math.min(xLength, rectangle.getWidth() - xOffset);
          double height = Math.min(yLength, rectangle.getHeight() - yOffset);
          // Don't forget to offset from the starting coordinates of the
          // intersection bounding box
          Rectangle2D tileRect =
            new Rectangle2D.Double(rectangle.getMinX() + xOffset,
                                   rectangle.getMinY() + yOffset,
                                   width, height);
          // Now that we have a rectangle for the tile, we can figure out
          // whether it is actually in the area
          if(area.intersects(tileRect)) {
            // If it is in the area, let's make a new tile
            tiles[x][y] = new SquareTile(tileRect, x, y, numberOfTiles);
            idToTiles.add(tiles[x][y]);
            numberOfTiles++;
          }
        }
      }
    }
  /**
   * @author Alexander Humphry
   * Create the tiles with predefined polygons (4 lane road)
   */
  private void createPolyTiles(List<Point2D> points, int padding){
	  numberOfTiles = 0;
	  PolyTileStore store = new PolyTileStore(padding);//TODO
	  //add all tile definitions here, defined by adding points, points to be added manualy
	  for(PolyTile tile : store.getTiles()){
		  if(tile.getPolygon().intersects(area.getBounds2D())){
			  tiles[tile.getX()][tile.getY()] = tile;
			  idToTiles.add(tiles[tile.getX()][tile.getY()]);
			  numberOfTiles++;
		  }
	  }
  }

  /**
   * Identify the tiles that are on the edge of the area managed by this
   * tiled area.
   */
  private void identifyEdgeTiles() {
    // Now we must go through and discover which are edge tiles.  These are
    // simply the tiles that are not surrounded (in all 8 directions) by
    // other tiles
    for(int x = 0; x < xNum; x++) {
      for(int y = 0; y < yNum; y++) {
        // Don't process things that aren't tiles
        if(tiles[x][y] != null) {
          // If it's on the edge of the 2D array, it's definitely one
          if(x == 0 || y == 0 || x == xNum - 1 || y == yNum - 1) {
            tiles[x][y].setEdgeTile(true); // make it an edge tile
          } else {
            // These are guaranteed not to be on the edge of the 2D array
            // so we can just check around
            if(tiles[x - 1][y - 1] == null || // up, left
               tiles[x][y - 1] == null ||     // up
               tiles[x + 1][y - 1] == null || // up, right
               tiles[x - 1][y] == null ||     // left
               tiles[x + 1][y] == null ||     // right
               tiles[x - 1][y + 1] == null || // down, left
               tiles[x][y + 1] == null ||     // down
               tiles[x + 1][y + 1] == null) { // down, right
              // At least one of the surrounding spots doesn't have a tile
              tiles[x][y].setEdgeTile(true);
            }
          }
        }
      }
    }
  }

  //////////////////////////////////////////
  // PUBLIC METHODS (getters and setters)
  //////////////////////////////////////////

  /**
   * Get the area controlled by this tiled area
   *
   * @return the area controlled by this tiled area
   */
  public Area getArea() {
    return area;
  }

  /**
   * Get the number of tiles in the x-direction
   *
   * @return the number of tiles in the x-direction
   */
  public int getXNum() {
    return xNum;
  }

  /**
   * Get the number of tiles in the y-direction
   *
   * @return the number of tiles in the y-direction
   */
  public int getYNum() {
    return yNum;
  }

  /**
   * Get the length of a tile in the x-direction
   *
   * @return the length of a tile in the x-direction
   */
  public double getXLength() {
    return xLength;
  }

  /**
   * Get the length of a tile in the y-direction
   *
   * @return the length of a tile in the y-direction
   */
  public double getYLength() {
    return yLength;
  }

  /**
   * Get the tile at the (x,y) location in the grid
   *
   * @param x  the x-coordinate of the tile
   * @param y  the y-coordinate of the tile
   * @return  the tile; return null if the tile does not exist
   */
  public Tile getTile(int x, int y) {
    return tiles[x][y];
  }


  /**
   * Get the total number of tiles
   *
   * @return the total number of tiles
   */
  public int getNumberOfTiles() {
    return numberOfTiles;
  }


  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  /**
   * Whether or not the tile are squares.
   *
   * @return whether or not the tile are squares
   */
  public boolean areTilesSquare() {
    return yLength == xLength;
  }

  /**
   * Get a tile according to its id.
   *
   * @param id  the id of the tile
   * @return the tile
   */
  public Tile getTileById(int id) {
    return idToTiles.get(id);
  }

  /**
   * Get the list of tiles that are occupied by the given Shape.
   *
   * @param shape   the Shape for which to find occupied tiles
   * @return  the List of tiles that are occupied by the given Shape
   */
  public List<Tile> findOccupiedTiles(Shape shape) {
    // A place to store the answer
    List<Tile> occupiedTiles = new ArrayList<Tile>();
    // We only need to check the tiles that are within the bounding box
    Rectangle2D boundingBox = shape.getBounds2D();
    // Now find out the actual indices that this bounding box corresponds to.
    // We do this by first finding out what the relative coordinate is (by
    // subtracting the two values), then finding how this fits into our
    // 2D array of tiles, by dividing by the width of the rectangle, and
    // multiplying by the number of columns or rows.
    int firstColumn = 0;
    int lastColumn = xNum - 1;
    int firstRow = 0;
    int lastRow = yNum - 1;
    // only use short cut if 
    
	    firstColumn =
	      Math.max(0,
	               (int)((boundingBox.getMinX() - rectangle.getMinX()) /
	                     xLength));
	    lastColumn =
	      Math.min(xNum - 1,
	               (int)((boundingBox.getMaxX() - rectangle.getMinX()) /
	                     xLength));
	    firstRow =
	      Math.max(0,
	               (int)((boundingBox.getMinY() - rectangle.getMinY()) /
	                     yLength));
	    lastRow =
	      Math.min(yNum - 1,
	               (int)((boundingBox.getMaxY() - rectangle.getMinY()) /
	                     yLength));
	    
	    // Now go through all the potential tiles and find the ones that this
	    // shape intersects
	    lastColumn = xNum - 1;
	    lastRow = yNum - 1;
	    for(int c = firstColumn; c <= lastColumn; c++) {
	      for(int r = firstRow; r <= lastRow; r++) {
	        // If the tile exists, and it does intersect, add it to the list of
	        // tiles that are occupied
	        if(tiles[c][r] != null &&
	           testIntersection(shape,tiles[c][r].getShape())) {
	          occupiedTiles.add(tiles[c][r]);
	        }
	      }
    }
    return occupiedTiles;
  }
  /**
   * tests the intersection of two shapes, returns true if they intersect.
   * 
   * @author Alexander Humphry
   * @param shapeA
   * @param shapeB
   * @return whether the two shapes intersect
   */
  public boolean testIntersection(Shape shapeA, Shape shapeB){
	  Area areaA = new Area(shapeA);
	  areaA.intersect(new Area(shapeB));
	  return !areaA.isEmpty();
  }
  /**
   * @author Alexander Humphry
   * a tile with an irregular shape
   */
  public static class PolyTile implements Tile{
	  /** The area controlled by this tile. */
      private final Path2D polygon;
      /** the x-coordinate of this tile */
      private final int x;
      /** the y-coordinate of this tile */
      private final int y;
      /** the id of this tile */
      private final int id;
      /** whether or not a tile is on the edge */
      private boolean edgeTile = false;
	  
	  /**
	     * Create a tile.
	     *
	     * @param rectangle  the area of the tile
	     * @param x          the x-coordinate of the tile
	     * @param y          the y-coordinate of the tile
	     * @param id         the ID of the tile
	     */
  		PolyTile(Path2D polygon, int x, int y, int id){
  			this.polygon = polygon;
  			this.x = x;
  			this.y = y;
  			this.id = id;
  		}
  		/** Get the area controlled by this ReservationTile. 
  		 * @return polygon as a Path2D object
  		 */
  	    public Path2D getPolygon() {
  	      return polygon;
  	    }

  	    /** Get the x-coordinate of this tile 
  	     * @return tile x coordinate
  	     */
  	    public int getX() {
  	      return x;
  	    }

  	    /** Get the y-coordinate of this tile 
  	     * @return tile y coordinate
  	     */
  	    public int getY() {
  	      return y;
  	    }

  	    /** Get the id of this tile 
  	     * @return tile id
  	     */
  	    public int getId() {
  	      return id;
  	    }

  	    /** Whether or not this tile is on the edge 
  	     * @return is an edge tile
  	     */
  	    public boolean isEdgeTile() {
  	      return edgeTile;
  	    }

  	    /**
  	     * Set whether or not this tile is on the edge.
  	     *
  	     * @param edgeTile  whether or not this tile is on the edge
  	     */
  	    public void setEdgeTile(boolean edgeTile) {
  	    	
  	      this.edgeTile = edgeTile;
  	    }
		public Path2D getShape() {
			return polygon;
		}
  }
  
  public class PolyTileStore{
	  
	  private ArrayList<PolyTile> polyTileStore = new ArrayList<PolyTile>();
	  
	  public PolyTileStore(int padding){
		  populateDefault(padding);
	  }
	  public PolyTileStore(List<Point2D> points, double minDist){
		  generateGraph(points, minDist);
	  }
	  
	  public int getNumberOfPolyTiles(){
		  return polyTileStore.size();
	  }
	  public ArrayList<PolyTile> getTiles(){
		  return polyTileStore;
	  }
	  private void generateGraph(List<Point2D> points, double minDist){
		  Voronoi diagram = new Voronoi(minDist);
		  Vertex[] vertices = convertPoints(points);
		  List<GraphEdge> graph = diagram.generateVoronoi(vertices);
		  Map<Vertex,Path2D> polygons = diagram.makeVoronoiPolygons(vertices, graph);
		  polygons = diagram.generateBorderPolygons(polygons);
		  ArrayList<Vertex> polygonList = new ArrayList<Vertex>(polygons.keySet());
		  
		  int i = 0;
		  Double x;
		  Double y;
		  for(Vertex polygon : polygonList){
			  x = polygon.xPos - rectangle.getMinX();
			  y = polygon.yPos - rectangle.getMinY();
			  polyTileStore.add(new PolyTile(polygons.get(polygon),x.intValue() , y.intValue(), i));
			  i++;
		  }
	  }
	  
	  private Vertex[] convertPoints(List<Point2D> points) {
		  Vertex[] vertices = new Vertex[points.size()];//TODO double check functionality, should convert points list to array
		  for(int i = 0; i < points.size(); i++){
			  vertices[i] = new Vertex(points.get(i).getX(),points.get(i).getY());
		  }
		  return vertices;
	}
	public void populateDefault(int padding){
		  //add tiles manually here
		  //voronoi code test
		  Voronoi interVoronoi = new Voronoi(0.5);
		  //test points location
		  Vertex[] points = generatePoints(4, padding);
		  
		  List<GraphEdge> graph= interVoronoi.generateVoronoi(points);
		  Map<Vertex,Path2D> polygons= interVoronoi.makeVoronoiPolygons(points, graph);
		  polygons = interVoronoi.generateBorderPolygons(polygons);
		  
		  ArrayList<Vertex> polygonList = new ArrayList<Vertex>(polygons.keySet());
		  if(!polygons.isEmpty()){
			  Path2D testPath = polygons.get(polygonList.get(0));
			  testPath.getBounds2D();
		  }
		  if(!graph.isEmpty()){
			  graph.get(0);
		  }

		  int i = 0;
		  Double x;
		  Double y;
		  polyTileStore.clear();
		  for(Vertex polygon : polygonList){
			  x = polygon.xPos - rectangle.getMinX();
			  y = polygon.yPos - rectangle.getMinY();
			  polyTileStore.add(new PolyTile(polygons.get(polygon),x.intValue() , y.intValue(), i));
			  i++;
		  }
		  //test poly 1
		  
		  
		  /*
		  Path2D subject = new Path2D.Double();
		  subject.moveTo(rectangle.getMinX() + 0, rectangle.getMinY() + 0);
		  subject.lineTo(rectangle.getMinX() + 11.2, rectangle.getMinY() + 0);
		  subject.lineTo(rectangle.getMinX() + 11.2, rectangle.getMinY() + 11.2);
		  subject.lineTo(rectangle.getMinX() + 0, rectangle.getMinY() + 11.2);
		  subject.lineTo(rectangle.getMinX() + 0, rectangle.getMinY() + 0);
		  subject.closePath();
		  
		  polyTileStore.add(new PolyTile(subject,6,6,0));
		  //test poly 2, x - y
		  subject = new Path2D.Double();
		  subject.moveTo(rectangle.getMinX() + 11.2, rectangle.getMinY() + 0);
		  subject.lineTo(rectangle.getMaxX(), rectangle.getMinY() + 0);
		  subject.lineTo(rectangle.getMaxX(), rectangle.getMinY() + 11.2);
		  subject.lineTo(rectangle.getMinX() + 11.2, rectangle.getMinY() + 11.2);
		  subject.lineTo(rectangle.getMinX() + 11.2, rectangle.getMinY() + 0);
		  subject.closePath();
		  
		  polyTileStore.add(new PolyTile(subject,17,6,1));
		  //test poly 3
		  subject = new Path2D.Double();
		  subject.moveTo(rectangle.getMinX() + 0, rectangle.getMinY() + 11.2);
		  subject.lineTo(rectangle.getMinX() + 11.2, rectangle.getMinY() + 11.2);
		  subject.lineTo(rectangle.getMinX() + 11.2, rectangle.getMaxY());
		  subject.lineTo(rectangle.getMinX() + 0, rectangle.getMaxY());
		  subject.lineTo(rectangle.getMinX() + 0, rectangle.getMinY() + 11.2);
		  subject.closePath();
		  
		  polyTileStore.add(new PolyTile(subject,6,17,2));
		  //test poly 4
		  subject = new Path2D.Double();
		  subject.moveTo(rectangle.getMinX() + 11.2, rectangle.getMinY() + 11.2);
		  subject.lineTo(rectangle.getMaxX(), rectangle.getMinY() + 11.2);
		  subject.lineTo(rectangle.getMaxX(), rectangle.getMaxY());
		  subject.lineTo(rectangle.getMinX() + 11.2, rectangle.getMaxY());
		  subject.lineTo(rectangle.getMinX() + 11.2, rectangle.getMinY() + 11.2);
		  subject.closePath();
		  
		  polyTileStore.add(new PolyTile(subject,17,17,3));*/
	  }
	/**
	 * used to store the conflict points of the voronoi diagram
	 * @return
	 */
	private Vertex[] generatePoints(int set, int padding) {
		  ArrayList<Vertex> pointsList = new ArrayList<Vertex>();
		  Vertex point;
		  //test1
		  if(set == 1){
			  //point 1
			  point = generateGridVertex(0.0,0.0,true,true);
			  pointsList.add(point);
			  //point 2
			  point = generateGridVertex(0.0,0.0,true,false);
			  pointsList.add(point);
			  //point 3
			  point = generateGridVertex(0.0,0.0,false,true);
			  pointsList.add(point);
			  //point 4
			  point = generateGridVertex(0.0,0.0,false,false);
			  pointsList.add(point);
			  //point 5
			  point = generateGridVertex(rectangle.getWidth()/ 2,rectangle.getHeight()/ 2,true,true);
			  pointsList.add(point);
			  //test2
		  } else if(set == 2){
			  //point 1
			  point = generateGridVertex(rectangle.getWidth()/ 2,0.0,true,true);
			  pointsList.add(point);
			  //point 2
			  point = generateGridVertex(rectangle.getWidth()/ 2,0.0,true,false);
			  pointsList.add(point);
			  //point 3
			  point = generateGridVertex(0.0,rectangle.getHeight()/ 2,true,true);
			  pointsList.add(point);
			  //point 4
			  point = generateGridVertex(0.0,rectangle.getHeight()/ 2,false,true);
			  pointsList.add(point);
			  //test3
		  } else if(set == 3){
			  //point 1
			  point = generateGridVertex(rectangle.getWidth()/ 2,0.0,true,true);
			  pointsList.add(point);
			  //point 2
			  point = generateGridVertex(rectangle.getWidth()/ 2,0.0,true,false);
			  pointsList.add(point);
			  //point 3
			  point = generateGridVertex(0.0,rectangle.getHeight()/ 2,true,true);
			  pointsList.add(point);
			  //point 4
			  point = generateGridVertex(0.0,rectangle.getHeight()/ 2,false,true);
			  pointsList.add(point);
			  //point 5
			  point = generateGridVertex(rectangle.getWidth()/ 2,rectangle.getHeight()/ 2,true,true);
			  pointsList.add(point);
		  } else if(set == 4){
			  //curve intersection points
			  //point 1
			  point = generateGridVertex(6.9,rectangle.getHeight()/ 2,true,true);
			  pointsList.add(point);
			  //point 2
			  point = generateGridVertex(-6.9,rectangle.getHeight()/ 2,false,true);
			  pointsList.add(point);
			  //point 3
			  point = generateGridVertex(rectangle.getWidth()/ 2,6.9,true,true);
			  pointsList.add(point);
			  //point 4
			  point = generateGridVertex(rectangle.getWidth()/ 2,-6.9,true,false);
			  pointsList.add(point);
		  //edge points
			  //minx
			  //point1
			  point = generateGridVertex(0.0,5.8,true,true);
			  pointsList.add(point);
			  //point 2
			  point = generateGridVertex(0.0,9.4,true,true);
			  pointsList.add(point);
			  //point 3
			  point = generateGridVertex(0.0,13.0,true,true);
			  pointsList.add(point);
			  //point 4
			  point = generateGridVertex(0.0,16.6,true,true);
			  pointsList.add(point);
			  //maxx
			  //point1
			  point = generateGridVertex(0.0,5.8,false,true);
			  pointsList.add(point);
			  //point 2
			  point = generateGridVertex(0.0,9.4,false,true);
			  pointsList.add(point);
			  //point 3
			  point = generateGridVertex(0.0,13.0,false,true);
			  pointsList.add(point);
			  //point 4
			  point = generateGridVertex(0.0,16.6,false,true);
			  pointsList.add(point);
			  //miny
			  //point1
			  point = generateGridVertex(5.8,0.0,true,true);
			  pointsList.add(point);
			  //point 2
			  point = generateGridVertex(9.4,0.0,true,true);
			  pointsList.add(point);
			  //point 3
			  point = generateGridVertex(13.0,0.0,true,true);
			  pointsList.add(point);
			  //point 4
			  point = generateGridVertex(16.6,0.0,true,true);
			  pointsList.add(point);
			  //maxy
			  //point1
			  point = generateGridVertex(5.8,0.0,true,false);
			  pointsList.add(point);
			  //point 2
			  point = generateGridVertex(9.4,0.0,true,false);
			  pointsList.add(point);
			  //point 3
			  point = generateGridVertex(13.0,0.0,true,false);
			  pointsList.add(point);
			  //point 4
			  point = generateGridVertex(16.6,0.0,true,false);
			  pointsList.add(point);
		  //inner conflict points
			  //frame 1
			  //point1
			  point = generateGridVertex(5.8,5.8,true,true);
			  pointsList.add(point);
			  //point 2
			  point = generateGridVertex(5.8,9.4,true,true);
			  pointsList.add(point);
			  //point 3
			  point = generateGridVertex(5.8,13.0,true,true);
			  pointsList.add(point);
			  //point 4
			  point = generateGridVertex(5.8,16.6,true,true);
			  pointsList.add(point);
			  //frame 2
			  //point1
			  point = generateGridVertex(9.4,5.8,true,true);
			  pointsList.add(point);
			  //point 2
			  point = generateGridVertex(9.4,9.4,true,true);
			  pointsList.add(point);
			  //point 3
			  point = generateGridVertex(9.4,13.0,true,true);
			  pointsList.add(point);
			  //point 4
			  point = generateGridVertex(9.4,16.6,true,true);
			  pointsList.add(point);
			  //frame 3
			  //point1
			  point = generateGridVertex(13.0,5.8,true,true);
			  pointsList.add(point);
			  //point 2
			  point = generateGridVertex(13.0,9.4,true,true);
			  pointsList.add(point);
			  //point 3
			  point = generateGridVertex(13.0,13.0,true,true);
			  pointsList.add(point);
			  //point 4
			  point = generateGridVertex(13.0,16.6,true,true);
			  pointsList.add(point);
			  //frame 4
			//point1
			  point = generateGridVertex(16.6,5.8,true,true);
			  pointsList.add(point);
			  //point 2
			  point = generateGridVertex(16.6,9.4,true,true);
			  pointsList.add(point);
			  //point 3
			  point = generateGridVertex(16.6,13.0,true,true);
			  pointsList.add(point);
			  //point 4
			  point = generateGridVertex(16.6,16.6,true,true);
			  pointsList.add(point);
			  //add other points
			  pointsList.addAll(generateGridVertecies(padding));
		  }
		  
		  //convert to array
		  Vertex[] points = new Vertex[pointsList.size()];
		  pointsList.toArray(points);
		return points;
	}
	ArrayList<Vertex> conflictGenerator(int laneNumber, double laneSpacing, double laneCenterPos){
		//TODO
		
		return null;
	}
  }
  private Vertex generateGridVertex(Double x, Double y, boolean useMinX, boolean useMinY){
	  if(useMinX){
		  if(useMinY){
			  return new Vertex(rectangle.getMinX() + x, rectangle.getMinY() + y);
		  } else {
			  return new Vertex(rectangle.getMinX() + x, rectangle.getMaxY() + y);
		  }
	  } else {
		  if(useMinY){
			  return new Vertex(rectangle.getMaxX() + x, rectangle.getMinY() + y);
		  }	else {
			  return new Vertex(rectangle.getMaxX() + x, rectangle.getMaxY() + y);
		  }
	  }
  }
  private ArrayList<Vertex> generateGridVertecies(int pointsInLimits){
	  ArrayList<Vertex> vertecies = new ArrayList<Vertex>();
	  if(pointsInLimits == 0){
		  return vertecies;
	  }
	  double intervalx = (rectangle.getMaxX() - rectangle.getMinX())/(pointsInLimits + 1);
	  double intervaly = (rectangle.getMaxY() - rectangle.getMinY())/(pointsInLimits + 1);
	  
	  for(int i = 0; i < pointsInLimits ; i++){
		  for(int j = 0; j < pointsInLimits; j++){
			  vertecies.add(generateGridVertex(intervalx*i,intervaly*j, true, true));
		  }
	  }
	  return vertecies;
  }
}
