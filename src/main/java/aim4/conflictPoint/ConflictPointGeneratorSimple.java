package aim4.conflictPoint;

import java.awt.geom.Arc2D;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

import aim4.im.Intersection;
import aim4.im.IntersectionManager;
import aim4.map.BasicMap;
import aim4.map.Road;
import aim4.map.lane.Lane;

public class ConflictPointGeneratorSimple implements ConflictPointGenerator{
	
	BasicMap map;
	Intersection inter;
	
	public ConflictPointGeneratorSimple(BasicMap map){
		this.map = map;
	}
	public ConflictPointGeneratorSimple(){
		
	}
	public List<Point2D> generateConflictPoints(Intersection i, boolean restrictedTurning) {

		inter = i;
		ArrayList<Road> originRoads = new ArrayList<Road>(i.getEntryRoads());
		ArrayList<Road> destinationRoads = new ArrayList<Road>(i.getExitRoads());
		
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
				if(pathIsStraight(oRoad.getIndexLane().getTerminalHeading(), dRoad.getIndexLane().getInitialHeading()) || oRoad.getDual().equals(dRoad)){
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
		Point2D start1 = oLane.getStartPoint();
		Point2D end1 = dLane.getEndPoint();
		//bring relevent point to intersection threshold
		Point2D start, end;
		
		//if vertical
		if(oLane.getEndPoint().getX() == oLane.getStartPoint().getX()){
			if(start1.getY() == 0){
				start = new Point2D.Double(start1.getX(),this.inter.getBoundingBox().getMinY());
			}else{
				start = new Point2D.Double(start1.getX(),this.inter.getBoundingBox().getMaxY());
			}
			//end is horizontal
			if(end1.getX() == 0){
				end = new Point2D.Double(this.inter.getBoundingBox().getMinX(),end1.getY());
			} else {
				end = new Point2D.Double(this.inter.getBoundingBox().getMaxX(),end1.getY());
			}
		} else {
			//start is horizontal
			if(start1.getX() == 0){
				start = new Point2D.Double(this.inter.getBoundingBox().getMinX(),start1.getY());
			}else{
				start = new Point2D.Double(this.inter.getBoundingBox().getMaxX(),start1.getY());
			}
			//end is vertical
			if(end1.getY() == 0){
				end = new Point2D.Double(end1.getX(),this.inter.getBoundingBox().getMinY());
			} else {
				end = new Point2D.Double(end1.getX(),this.inter.getBoundingBox().getMaxY());
			}
		}
		
		
		Arc2D result = new Arc2D.Double();
		
		result.setArcByCenter((start.getX() + end.getX())/2, 
				(start.getY() + end.getY())/2, 
				Math.abs(start.getY() - end.getY()), 
				oLane.getTerminalHeading()*180/Math.PI, 
				dLane.getInitialHeading()*180/Math.PI, 
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
	//assumes lines are ither horizontal or vertical
	private ArrayList<Point2D> getLineIntersections(ArrayList<Line2D> lines){
		
		ArrayList<Point2D> result = new ArrayList<Point2D>();
		
		Line2D loi;
		//double a1;
		//double b1;
		
		Line2D los;
		//double a2;
		//double b2;
		
		
		boolean horizontal1;
		boolean horizontal2;
		
		Point2D poi;
		double poix;
		double poiy;
		
		for(int i = 0; i < lines.size(); i++){
			loi = lines.get(i);
			/*
			if(loi.getX1() == loi.getX2()){
				
			}
			a1 = (loi.getY2() - loi.getY1())/(loi.getX2()-loi.getX1());
			b1 = loi.getY1() - (a1 * loi.getX1());*/
			if(loi.getX1() == loi.getX2()){
				horizontal1 = true;
			} else {
				horizontal1 = false;
			}
			for(int j = i + 1; j < lines.size(); j++){
				los = lines.get(j);
				if(los.getX1() == los.getX2()){
					horizontal2 = true;
				} else {
					horizontal2 = false;
				}
				if(horizontal1 == horizontal2){
					continue;
				}
				if(horizontal1){
					poix = loi.getX1();
					poiy = los.getY1();
				} else {
					poix = los.getX1();
					poiy = loi.getY1();
				}
				/*a2 = (los.getY2() - los.getY1())/(los.getX2()-los.getX1());
				//check if parallel, if so, next calc
				if(a1 == a2){
					continue;
				}
				
				b2 = los.getY1() - (a2 * los.getX1());*/
				//calc x point
				/*poix = (b2 - b1)/(a1 - a2);
				poiy = a1 * poix + b1;*/
				poi = new Point2D.Double(poix, poiy);
				result.add(poi);
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
		ArrayList<Point2D> results = new ArrayList<Point2D>();

		Point2D cand;
		
		//arc and line intersection
		for(Arc2D arc : arcs){

			for(Line2D line : lines){

				if(!willIntersect(line,arc)){
					continue;
				}

				cand = getLikelyPoint(line, arc);
				if(cand != null){
					results.add(cand);
				}
			}
		}
		return results;
	}
	
	private Point2D getLikelyPoint(Line2D line, Arc2D arc){
		
		
		Point2D center = getArcFocusPoint(arc); // h and k
		double rad = Math.abs(arc.getStartPoint().getX() - arc.getEndPoint().getX()); // r
		
		
		double x1, x2, y1, y2, y3, y4, a, b, h, k;
		double aq,bq,cq;
		
		ArrayList<Point2D> cand = new ArrayList<Point2D>();
		
		boolean vertical = false;
		if(line.getX1() == line.getX2()){
			vertical = true;
		}
		
		if(vertical){
			x1 = line.getX1();
			y1 = center.getY() + Math.sqrt(rad*rad - (x1 - center.getX())*(x1 - center.getX()));
			y2 = center.getY() - Math.sqrt(rad*rad - (x1 - center.getX())*(x1 - center.getX()));
			cand.add(new Point2D.Double(x1,y1));
			cand.add(new Point2D.Double(x1,y2));
			if(this.inter.getBoundingBox().contains(cand.get(0))){
				return cand.get(0);
			}
			if(this.inter.getBoundingBox().contains(cand.get(1))){
				return cand.get(1);
			}
			return null;
		}
		//if not vertical
		a = (line.getY1() - line.getY2())/(line.getX1()-line.getX2());
		b = line.getY1() - a * line.getX1();
		h = center.getX();
		k = center.getY();
		aq = (1 + a*a);
		bq = 2*a*b - 2*h - 2*a*k;
		cq = h*h + k*k - 2*b*k + b*b-rad*rad;
		
		x1 = (-bq + Math.sqrt(bq*bq - 4*aq*cq))/(2*aq);
		y1 = center.getY() + Math.sqrt(rad*rad - (x1 - center.getX())*(x1 - center.getX()));
		y2 = center.getY() - Math.sqrt(rad*rad - (x1 - center.getX())*(x1 - center.getX()));
		
		cand.add(new Point2D.Double(x1,y1));
		cand.add(new Point2D.Double(x1,y2));
		
		x2 = (-bq - Math.sqrt(bq*bq - 4*aq*cq))/(2*aq);
		y3 = center.getY() + Math.sqrt(rad*rad - (x2 - center.getX())*(x2 - center.getX()));
		y4 = center.getY() - Math.sqrt(rad*rad - (x2 - center.getX())*(x2 - center.getX()));
		
		cand.add(new Point2D.Double(x2,y3));
		cand.add(new Point2D.Double(x2,y4));
		
		for(Point2D candidate : cand){
			if(this.inter.getBoundingBox().contains(candidate)){
				return candidate;
			}
		}
		return null;
	}
	
	private boolean willIntersect(Line2D line, Arc2D arc){
		boolean isVertical = false;
		if(line.getX1() == line.getX2()){
			isVertical = true;
		}
		if(isVertical){
			if(line.getX1() < arc.getMaxX() && line.getX1() > arc.getMinX()){
				return true;
			}
		} else {
			if(line.getY1() < arc.getMaxY() && line.getY1() > arc.getMinY()){
				return true;
			}
		}
		return false;
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
