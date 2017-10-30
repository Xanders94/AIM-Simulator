package aim4.conflictPoint;

import java.awt.geom.Arc2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import aim4.im.Intersection;
import aim4.im.IntersectionManager;
import aim4.map.BasicMap;
import aim4.map.Road;
import aim4.map.lane.Lane;

public class ConflictPointGeneratorSimple implements ConflictPointGenerator{
	
	BasicMap map;
	Intersection inter;
	
	ConflictPointGeneratorSimple(BasicMap map){
		this.map = map;
	}
	public List<Point2D> generateConflictPoints(IntersectionManager im,
			double pointMergeDist, boolean restrictedTurning) {

		inter = im.getIntersection();
		ArrayList<Road> originRoads = new ArrayList<Road>(im.getIntersection().getEntryRoads());
		ArrayList<Road> destinationRoads = new ArrayList<Road>(im.getIntersection().getExitRoads());
		
		ArrayList<Line2D> straightPaths = getStraightPaths(originRoads, destinationRoads);
		ArrayList<Arc2D> curvedPaths = getCurvedPaths(originRoads, destinationRoads, restrictedTurning);
		
		ArrayList<Point2D> conflictPoints = getConflictPoints(straightPaths, curvedPaths);
		
		
		
		return conflictPoints;
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
	private ArrayList<Line2D> getLaneLines(Road oRoad, Road dRoad) {
		ArrayList<Line2D> result = new ArrayList<Line2D>();
		for(int i = 0; i < Math.min(oRoad.getLanes().size(), dRoad.getLanes().size()); i++){
			result.add(new Line2D.Double(oRoad.getLanes().get(i).getEndPoint(),dRoad.getLanes().get(i).getStartPoint()));
		}
		return result;
	}
	private ArrayList<Arc2D> getCurvedPaths(ArrayList<Road> oRoads,
			ArrayList<Road> dRoads, boolean restrictedTurning) {
		
		ArrayList<Arc2D> result = new ArrayList<Arc2D>();
		
		for(Road oRoad : oRoads){
			for(Road dRoad : dRoads){
				//detect if straight turn
				if(pathIsStraight(oRoad.getIndexLane().getTerminalHeading(), dRoad.getIndexLane().getInitialHeading())){
					continue;
				}
				//detect if restricted turning and is a right hand turn
				if(restrictedTurning && !isLeftHandTurn(oRoad.getIndexLane().getTerminalHeading(), dRoad.getIndexLane().getInitialHeading())){
					continue;
				}
				for(int i = 0; i < Math.min(oRoad.getLanes().size(), dRoad.getLanes().size()); i++){
					if(restrictedTurning && i > 0){
						break;
					}
					result.add(generateArc(oRoad.getLanes().get(i),dRoad.getLanes().get(i)));
				}
			}
		}
		
		return result;
	}
	/**
	 * generates and arch based on an origin and destination lane set
	 * 
	 * @param oLane
	 * @param dLane
	 * @return
	 */
	private Arc2D generateArc(Lane oLane, Lane dLane){
		Point2D start = oLane.getEndPoint();
		Point2D end = dLane.getStartPoint();
		Arc2D result = new Arc2D.Double();
		
		result.setArcByCenter((start.getX() + end.getX())/2, 
				(start.getY() + end.getY())/2, 
				Math.abs(start.getY() - end.getY()), 
				oLane.getTerminalHeading(), 
				dLane.getInitialHeading(), 
				Arc2D.OPEN);
		
		return result;
	}
	
	private ArrayList<Point2D> getConflictPoints(
			ArrayList<Line2D> straightPaths, ArrayList<Arc2D> curvedPaths) {
		
		ArrayList<Point2D> result = new ArrayList<Point2D>();
		
		//add straight line end points
		for(Line2D line : straightPaths){
			result.add(line.getP1());
			result.add(line.getP2());
		}
		
		//find intersection of straight paths
		
		result.addAll(getLineIntersections(straightPaths));
		
		//find intersection of curved paths
		
		result.addAll(getArcIntersections(curvedPaths,straightPaths));
		
		//merge points if too close
		
		return result;
	}
	private ArrayList<Point2D> getLineIntersections(ArrayList<Line2D> lines){
		
		ArrayList<Point2D> result = new ArrayList<Point2D>();
		
		Line2D loi;
		double a1;
		double b1;
		
		Line2D los;
		double a2;
		double b2;
		
		Point2D poi;
		double poix;
		double poiy;
		
		for(int i = 0; i < lines.size(); i++){
			loi = lines.get(i);
			a1 = (loi.getY2() - loi.getY1())/(loi.getX2()-loi.getX1());
			b1 = loi.getY1() - (a1 * loi.getX1());
			for(int j = i + 1; j < lines.size(); j++){
				los = lines.get(j);
				a2 = (los.getY2() - los.getY1())/(los.getX2()-los.getX1());
				//check if parallel, if so, next calc
				if(a1 == a2){
					continue;
				}
				b2 = los.getY1() - (a2 * los.getX1());
				//calc x point
				poix = (b2 - b1)/(a1 - a2);
				poiy = a1 * poix + b1;
				poi = new Point2D.Double(poix, poiy);
				if(loi.contains(poi)&&los.contains(poi)){
					result.add(poi);
				}
			}
		}
		return result;
	}
	
	private ArrayList<Point2D> getArcIntersections(ArrayList<Arc2D> arcs, ArrayList<Line2D> lines){
		
		ArrayList<Point2D> results = new ArrayList<Point2D>();
		
		results.addAll(getIntersectionArcLine(arcs,lines));
		
		//arc and arc intersection
		
		results.addAll(getIntersectionArcArc(arcs));
		
		return results;
	}
	private ArrayList<Point2D> getIntersectionArcLine(ArrayList<Arc2D> arcs, ArrayList<Line2D> lines){
		// TODO Auto-generated method stub
		ArrayList<Point2D> results = new ArrayList<Point2D>();
		
		//arc params
		Point2D center;
		double rad;
		
		//line params
		double x1;
		double x2;
		double y1;
		double y2;
		
		//calc params
		double dx;
		double dy;
		double dr;
		double dd;
		double xx;
		double yy;
		
		
		//arc and line intersection
		for(Arc2D arc : arcs){
			center = getArcFocusPoint(arc);
			rad = Math.abs(arc.getStartPoint().getX() - arc.getEndPoint().getX());
			for(Line2D line : lines){
				//check if line intersects arc
				if(!arc.intersects(line.getBounds2D())){
					continue;
				}
				x1 = line.getX1() - center.getX();
				x2 = line.getX2() - center.getX();
				y1 = line.getY1() - center.getY();
				y2 = line.getY2() - center.getY();
				
				dx = x2 - x1;
				dy = y2 - y1;
				dr = Math.sqrt((dx*dx)+(dy*dy));
				dd = x1*y2 - x2*y1;
				results.add(getLikelyPoint(dd,dy,dx,rad,dr, center.getX(), center.getY(),line));
			}
		}
		return results;
	}
	
	private Point2D getLikelyPoint(double dd, double dy, double dx, double r, double dr, double centerX, double centerY, Line2D line){
		ArrayList<Point2D> candidates = new ArrayList<Point2D>();
		Rectangle2D intersection = inter.getBoundingBox();
		double factor1, factor2;
		
		//calc factor 1 = sgn*(dy)*dx*sqrt(r^2*dr^2-dd^2)
		//calc factor 2 = |dy|*sqrt(r^2dr^2-dd^2)
		if(dy < 0){
			factor1 = -1 * dx * Math.sqrt(r*r*dr*dr - dd*dd);
		} else {
			factor1 = 1 * dx * Math.sqrt(r*r*dr*dr - dd*dd);
		}
		factor2 = Math.abs(dy)*Math.sqrt(r*r*dr*dr - dd*dd);
		
		//++
		candidates.add(new Point2D.Double((dd*dy+factor1)/(dr*dr),(-dd*dx+factor2)/(dr*dr)));
		//+-
		candidates.add(new Point2D.Double((dd*dy+factor1)/(dr*dr),(-dd*dx-factor2)/(dr*dr)));
		//-+
		candidates.add(new Point2D.Double((dd*dy-factor1)/(dr*dr),(-dd*dx+factor2)/(dr*dr)));
		//--
		candidates.add(new Point2D.Double((dd*dy-factor1)/(dr*dr),(-dd*dx-factor2)/(dr*dr)));
		
		for(Point2D cand : candidates){
			if(line.contains(cand)){
				if(intersection.contains(cand)){
					return cand;
				}
			}
		}
		return null;
	}
	//assumes base points at corner of intersection
	
	private ArrayList<Point2D> getIntersectionArcArc(ArrayList<Arc2D> arcs){
		// TODO Auto-generated method stub
		
		ArrayList<Point2D> result = new ArrayList<Point2D>();
		
		//circle 1
		Point2D r1;
		double rr1;
		
		//circle 2
		Point2D r2;
		double rr2;
		
		//calc params
		double dist; // distance between curve centers
		double x; // distance between circle 1 center and relative intersection x coordinate
		double y; // double distance perpendicular to line connecting circle 1 and 2, used to calc y coord of intersection point
		
		
		for(int i = 0; i < arcs.size(); i++){
			r1 = getArcFocusPoint(arcs.get(i));
			rr1 = Math.abs(arcs.get(i).getStartPoint().getX() - arcs.get(i).getEndPoint().getX());
			for(int j = i + 1; j < arcs.size(); j++){
				//if start and end angles match between arcs, skip as no intersection
				if(arcs.get(i).getAngleStart() == arcs.get(j).getAngleStart() && 
						arcs.get(i).getAngleExtent() == arcs.get(j).getAngleExtent()){
					continue;
				}
				r2 = getArcFocusPoint(arcs.get(j));
				rr2 = Math.abs(arcs.get(j).getStartPoint().getX() - arcs.get(j).getEndPoint().getX());
				dist = r1.distance(r2);
				//if addition of radii between curves 
				if(rr1 + rr2 < dist){
					continue;
				}
				x = ((dist*dist) - (rr2*rr2) + (rr1*rr1))/(2*dist);
				y = Math.sqrt((rr1*rr1) - (x*x));
				if(r1.getX() == r2.getX()){
					if(r1.getY() < r2.getY()){
						result.add(getLikelyPoint(r1,x,y,true,true));
					} else {
						result.add(getLikelyPoint(r1,x,y,true,false));
					}
				} else {
					if(r1.getX() < r2.getX()){
						result.add(getLikelyPoint(r1,x,y,false,true));
					} else {
						result.add(getLikelyPoint(r1,x,y,false,false));
					}
				}
			}
		}
		
		return result;
	}
	//assumes original circle is right most or below
	private Point2D getLikelyPoint(Point2D circle1, double x, double y, boolean isVertical, boolean rightmostOrLowest){
		Rectangle2D intersection = inter.getBoundingBox();
		Point2D cand1, cand2;
		int h;
		if(rightmostOrLowest){
			h = 1;
		} else {
			h = -1;
		}

		if(isVertical){
			cand1 = new Point2D.Double(circle1.getX() + y*h, circle1.getY() + x);
			cand2 = new Point2D.Double(circle1.getX() + y*h, circle1.getY() - x);
			if(intersection.contains(cand1)){
				return cand1;
			}else{
				return cand2;
			}
		}else{
			cand1 = new Point2D.Double(circle1.getX() + x, circle1.getY() + y*h);
			cand2 = new Point2D.Double(circle1.getX() - x, circle1.getY() + y*h);
			if(intersection.contains(cand1)){
				return cand1;
			}else{
				return cand2;
			}
		}
	}
	
	//assuming start and end are cardinal directions
	private Point2D getArcFocusPoint(Arc2D arc){
		
		Point2D start = arc.getStartPoint();
		Point2D end = arc.getEndPoint();
		if(arc.getAngleStart() == 0 || arc.getAngleStart() == 180){
			return new Point2D.Double(end.getX(),start.getY());
		}else{
			return new Point2D.Double(start.getX(),end.getY());
		}
		
		
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
