package aim4.conflictPoint;

import java.awt.geom.Arc2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collection;

import aim4.im.IntersectionManager;
import aim4.map.BasicMap;
import aim4.map.Road;
import aim4.map.lane.Lane;


//not completed, need to construct arcs
public class ConflictPointGeneratorComplex implements ConflictPointGenerator {
	//list of intersections for road objects
	BasicMap map;
	
	ConflictPointGeneratorComplex(BasicMap map){
		this.map = map;
	}
	
	public ArrayList<Point2D> generateConflictPoints(IntersectionManager im, double pointMergeDist, boolean restrictedTurning){
		
		ArrayList<Road> originRoads = new ArrayList<Road>(im.getIntersection().getEntryRoads());
		ArrayList<Road> destinationRoads = new ArrayList<Road>(im.getIntersection().getExitRoads());
		
		ArrayList<Line2D> straightPaths = getStraightPaths(originRoads, destinationRoads);
		ArrayList<Arc2D> curvedPaths = getCurvedPaths(originRoads, destinationRoads, restrictedTurning);
		
		ArrayList<Point2D> conflictPoints = getConflictPoints(straightPaths, curvedPaths, pointMergeDist);
		
		
		
		return conflictPoints;
	}


	private ArrayList<Point2D> getConflictPoints(
			ArrayList<Line2D> straightPaths, ArrayList<Arc2D> curvedPaths, double mergeDistance) {
		
		
		return null;
	}

	private ArrayList<Arc2D> getCurvedPaths(ArrayList<Road> originRoads,
			ArrayList<Road> destinationRoads, boolean restrictedTurning) {
		
		ArrayList<Arc2D> paths = new ArrayList<Arc2D>();
		
		//if restricted turning, right most lane and left most lane produce curve only
		for(Road oRoad : originRoads){
			for(Road dRoad : destinationRoads){
				//if dest road is dual or straight ahead, try another destination road
				if(pathIsStraight(oRoad.getIndexLane().getTerminalHeading(), dRoad.getIndexLane().getInitialHeading()) || 
						oRoad.getDual().equals(dRoad)){
					continue;
				}
				//construct Arc
				//paths.addAll(constructArcs(oRoad,dRoad));
			}
		}
		
		return null;
	}

	private ArrayList<Line2D> getStraightPaths(ArrayList<Road> originRoads,
			ArrayList<Road> destinationRoads) {
		
		ArrayList<Line2D> straightPaths = new ArrayList<Line2D>();
		
		for(Road oRoad : originRoads){
			// get opposite destination road
			for(Road dRoad : destinationRoads){
				if(oRoad.getIndexLane().getTerminalHeading() == dRoad.getIndexLane().getInitialHeading()){
					//get straight line paths where headings match
					straightPaths.addAll(getLaneLines(oRoad,dRoad));
					break;
				}
			}
		}
		
		return straightPaths;
	}

	private ArrayList<Line2D> getLaneLines(Road originRoad, Road destRoad) {
		
		ArrayList<Line2D> result = new ArrayList<Line2D>();
		Line2D tempLine;
		Point2D comparePoint;
		
		for(Lane oLane : originRoad.getLanes()){
			for(Lane dLane : destRoad.getLanes()){
				//if the temporary line heading equals the terminal heading of the origin lane, line is valid and added
				tempLine = new Line2D.Double(oLane.getEndPoint(),dLane.getStartPoint());
				comparePoint = getComparePoint(oLane.getEndPoint(), oLane.getTerminalHeading(), 0.1);
				if(tempLine.contains(comparePoint)){
					result.add(tempLine);
				}
			}
		}
			
		return result;
	}

	private Point2D getComparePoint(Point2D point, double terminalHeading,
			double d) {
		
		return new Point2D.Double(point.getX() + d * Math.cos(terminalHeading), point.getY() + d * Math.sin(terminalHeading));
	}
	
	/**
	   * @author Alexander Humphry
	   * 
	   * returns true if a left hand turn is required to travel from the initial heading to the final heading
	   * @param initHeading
	   * @param finalHeading
	   * @return
	   */
	  private boolean isLeftHandTurn(double initHeading, double finalHeading){
		  if(initHeading < Math.PI){
			  //if heading is in first 180 degrees
			  if(initHeading < finalHeading && finalHeading < initHeading + Math.PI){
				  return false;
			  }
			  return true;
		  } else {
			  if(initHeading - Math.PI < finalHeading && finalHeading < initHeading){
				  return true;
			  }
			  return false;
		  }
		  /* old code
		  if(initHeading > finalHeading){
			  return true;
		  }
		  if(initHeading + Math.PI < finalHeading){
			  return true;
		  }
		  return false;*/
	  }
	  /**
	   * @author Alexander Humphry
	   * 
	   * returns true if the path requires a turn of no more than PI * 0.1 radians
	   * @param initHeading
	   * @param finalHeading
	   * @return
	   */
	  private boolean pathIsStraight(double initHeading, double finalHeading){
		  if(Math.abs(initHeading - finalHeading) < Math.PI * 0.1){
			  return true;
		  }
		  if(Math.abs(initHeading + 2*Math.PI - finalHeading) < Math.PI * 0.1 || 
				  Math.abs(initHeading - 2*Math.PI - finalHeading) < Math.PI * 0.1){
			  return true;
		  }
		  return false;
	  }
	
}
