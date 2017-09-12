package aim4.im.v2i.batch;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.NavigableSet;
import java.util.PriorityQueue;

import aim4.config.Debug;
import aim4.im.TrackModel;
import aim4.im.v2i.RequestHandler.BatchModeRequestHandler.IndexedProposal;
import aim4.map.Road;
import aim4.map.SpawnPoint;
import aim4.msg.v2i.Request.Proposal;

public class WalletBasedReordering implements ReorderingStrategy {

	 /**
	   * The time period between the processing times.
	   */
	  public static final double DEFAULT_PROCESSING_INTERVAL = 2.0;  // seconds

	  /**
	   * The estimated time this reordering strategy takes to compute the
	   * reordering plus the time the intersection manager takes to send
	   * the confirm and reject messages.
	   */
	  private static final double COMP_COMM_DELAY = 0.05; // seconds

	  /**
	   * The amount of lookahead (the period of time between the end time of the
	   * last processed batch and the target batch).
	   */
	  private static final double LOOKAHEAD_TIME = 12.0;  // seconds //default 3.0 seconds

	  /**
	   * The amount of time of a batch.
	   */
	  private static final double BATCH_INTERVAL = DEFAULT_PROCESSING_INTERVAL;


	  /////////////////////////////////
	  // PRIVATE FIELDS
	  /////////////////////////////////

	  /**
	   * The next processing time for the next batch.
	   */
	  private double nextProcessingTime;

	  /**
	   * The next proposal deadline for the next batch.
	   */
	  private double nextProposalDeadline;


	  /**
	   * The time period between the processing times.
	   */
	  private double processingInterval = DEFAULT_PROCESSING_INTERVAL;
	  
	  private BidLaneTrack bidLaneTracker;


	  /////////////////////////////////
	  // CONSTRUCTORS
	  /////////////////////////////////

	  /**
	   * Create a road based reordering strategy
	   *
	   * @param processingInterval  the processing interval
	   */
	  public WalletBasedReordering(double processingInterval) {
	    this.processingInterval = processingInterval;
	    bidLaneTracker = new BidLaneTrack();
	  }

	  /////////////////////////////////
	  // PUBLIC METHODS
	  /////////////////////////////////

	  /**
	   * {@inheritDoc}
	   */
	  @Override
	  public void setInitialTime(double initTime) {
	    nextProcessingTime = initTime + processingInterval;
	    nextProposalDeadline = nextProcessingTime + COMP_COMM_DELAY;
	  }


	  /////////////////////////////////
	  // PUBLIC METHODS
	  /////////////////////////////////

	  /**
	   * {@inheritDoc}
	   */
	  @Override
	  public List<IndexedProposal> getBatch(double currentTime,
	                                        NavigableSet<IndexedProposal> queue,
	                                        TrackModel trackModel) {
		bidLaneTracker.refreshTraffic(queue, currentTime);
		bidLaneTracker.setAllToInactive();
	    List<IndexedProposal> proposals1 = selectProposals(currentTime, queue);
	    List<IndexedProposal> proposals2 = reorderProposals(proposals1);
	    List<IndexedProposal> proposals3 = bidLaneTracker.reorderProposals(proposals2, currentTime);
	    nextProcessingTime = currentTime + processingInterval;
	    nextProposalDeadline = nextProcessingTime + COMP_COMM_DELAY;
	    return proposals3;
	  }

	  /**
	   * {@inheritDoc}
	   */
	  @Override
	  public double getNextProcessingTime() {
	    return nextProcessingTime;
	  }

	  /**
	   * {@inheritDoc}
	   */
	  @Override
	  public double getNextProposalDeadline() {
	    return nextProposalDeadline;
	  }
	  
	  public BidLaneTrack getBidLaneTracker(){
		  return bidLaneTracker;
	  }

	  /////////////////////////////////
	  // PRIVATE METHODS
	  /////////////////////////////////

	  /**
	   * Select the set of proposals in a batch.
	   *
	   * @param currentTime  the current time
	   * @param queue        the message queue
	   * @return the set of proposals in a batch
	   */
	  private List<IndexedProposal> selectProposals(
	                                  double currentTime,
	                                  NavigableSet<IndexedProposal> queue) {
	    List<IndexedProposal> result = new LinkedList<IndexedProposal>();

	    double startTime = currentTime + LOOKAHEAD_TIME;
	    double endTime = startTime + BATCH_INTERVAL;

	    for(IndexedProposal iProposal : queue) {
	      Proposal proposal = iProposal.getProposal();
	      double arrivalTime = proposal.getArrivalTime();
	      if (arrivalTime < endTime) {    // exclude the arrivalTime == maxTime
	        result.add(iProposal);
	      } else {
	        // the remaining proposals in the queue have a larger arrival time
	    	// all proposals are required to total bids for the lane
	    	result.add(iProposal);
	        break;
	      }
	    }

	    return result;
	  }

	  /**
	   * Reorder a list of indexed proposals.
	   *
	   * @param iProposals a list of indexed proposals
	   * @return a reordered list of indexed proposals
	   */
	  private List<IndexedProposal> reorderProposals(
	                                             List<IndexedProposal> iProposals) {
		// a partition of the proposals according to the road of the arrival lane.
		    Map<Road,List<IndexedProposal>> partition =
		      new HashMap<Road,List<IndexedProposal>>();

		    for(IndexedProposal iProposal : iProposals) {
		      int laneId = iProposal.getProposal().getArrivalLaneID();
		      Road road = Debug.currentMap.getRoad(laneId);
		      if (partition.containsKey(road)) {
		        partition.get(road).add(iProposal);
		      } else {
		        List<IndexedProposal> l = new LinkedList<IndexedProposal>();
		        l.add(iProposal);
		        partition.put(road, l);
		      }
		    }

		    // combine the proposals according to the partition
		    List<IndexedProposal> result = new LinkedList<IndexedProposal>();

		    for(List<IndexedProposal> iProposals2 : partition.values()) {
		      result.addAll(iProposals2);
		    }

		    return result;
	  }
	}