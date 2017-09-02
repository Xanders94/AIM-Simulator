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
package aim4.map.destination;

import java.util.List;

import aim4.config.Debug;
import aim4.map.BasicMap;
import aim4.map.Road;
import aim4.map.lane.Lane;
import aim4.util.Util;

/**
 * The RandomDestinationSelector selects Roads uniformly at random, but will
 * not select a Road that is the dual of the starting Road.  This is to
 * prevent Vehicles from simply going back from whence they came.
 * 
 * @author Alexander Humphry
 */
public class RandomDestinationSelectorNoLaneCross implements DestinationSelector {

  /////////////////////////////////
  // PRIVATE FIELDS
  /////////////////////////////////

  /**
   * The Set of legal Roads that a vehicle can use as an ultimate destination.
   */
  private List<Road> destinationRoads;

  /////////////////////////////////
  // CLASS CONSTRUCTORS
  /////////////////////////////////

  /**
   * Create a new RandomDestinationSelector from the given Layout.
   *
   * @param layout the Layout from which to create the
   *               RandomDestinationSelector
   */
  public RandomDestinationSelectorNoLaneCross(BasicMap layout) {
    destinationRoads = layout.getDestinationRoads();
  }

  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  /**
   * {@inheritDoc}
   */
  @Override
  public Road selectDestination(Lane currentLane) {
    Road currentRoad = Debug.currentMap.getRoad(currentLane);
    Road dest =
      destinationRoads.get(Util.random.nextInt(destinationRoads.size()));
    while(dest.getDual() == currentRoad || !willNotCrossLanes(currentLane,dest)) {
      dest =
        destinationRoads.get(Util.random.nextInt(destinationRoads.size()));
    }
    return dest;
  }
  private boolean willNotCrossLanes(Lane currentLane, Road dest){
	  
	  //allow straight paths
	  if(pathIsStraight(currentLane.getInitialHeading(), dest.getIndexLane().getInitialHeading())){
		  return true;
	  }
	  //sim is left hand drive
	  //if there is a left lane, exclude right hand turns
	  //else exclude left hand turns
	  if(currentLane.hasLeftNeighbor()){
		  if(isLeftHandTurn(currentLane.getInitialHeading(), dest.getIndexLane().getInitialHeading())){
			  return false;
		  } else {
			  return true;
		  }
	  }else{
		  if(isLeftHandTurn(currentLane.getInitialHeading(), dest.getIndexLane().getInitialHeading())){
			  return true;
		  }else{
			  return false;
		  }
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
