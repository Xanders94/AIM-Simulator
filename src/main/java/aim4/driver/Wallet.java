package aim4.driver;

import java.util.Random;

public class Wallet {
	
	  /**
	   * driver's current bid over intersection
	   */
	  private double bid;
	  
	  /**
	   * driver's wallet containing total funds for the total trip
	   */
	  private double totalFundsAvailable;
	
	  /**
	   * intersection in trip left to traverse before destination
	   */
	  private int intersectionsInTrip;
	  
	  /**
	   * a blank constructor, sets all values in the wallet to zero
	   */
	  private Random rand;
	  /**
	   * the minimum randomly generated bid value
	   */
	  private static final double min = 0.05;
	  /**
	   * the maximum randomly generated bid value
	   */
	  private static final double max = 0.50;
	  
	  /**
	   * Generates a wallet with a random starting balance to be used over a single intersection.
	   */
	  public Wallet(){
		  rand = new Random();
		  totalFundsAvailable = randomRange(min,max);
		  bid = totalFundsAvailable;
		  intersectionsInTrip = 1;
		  
	  }
	  /**
	   * Generates a wallet with a random starting balance to be used over a given number of intersections.
	   * @param intersectionsInTrip Sets the number of intersections expected to use this wallet
	   * @param surplusFactor Sets the factor which increases or decreases the totalFundsAvailable after it is used to generate an initial bid. 0.5 translates a 50% reduction in the generated totalFundsAvailable, 1.5 translates to a 50% increase in the generated total funds available, 1 denotes no change. this can increase or decrease the total funds available beyond the default range. 
	   */
	  public Wallet(int intersectionsInTrip, double surplusFactor) {
		  rand = new Random();
		  totalFundsAvailable = randomRange(min,max);
		  bid = totalFundsAvailable / intersectionsInTrip;
		  totalFundsAvailable = totalFundsAvailable * surplusFactor;
		  this.intersectionsInTrip = intersectionsInTrip;
	  }
	  /**
	   * generates a wallet with a random starting balance within a given range.
	   * @param minRange the smallest possible value that may be generated
	   * @param maxRange the largest possible value that may be generated
	   */
	  public Wallet(double minRange, double maxRange) {
		  rand = new Random();
		  totalFundsAvailable = randomRange(minRange,maxRange);
		  bid = totalFundsAvailable;
		  intersectionsInTrip = 1;
	  }
	  /**
	   * generates a wallet with a random starting balance within a given range over a given number of intersections.
	   * @param minRange the smallest possible value that may be generated
	   * @param maxRange the largest possible value that may be generated
	   * @param intersectionsInTrip Sets the number of intersections expected to use this wallet
	   * @param surplusFactor Sets the factor which increases or decreases the totalFundsAvailable after it is used to generate an initial bid. 0.5 translates a 50% reduction in the generated totalFundsAvailable, 1.5 translates to a 50% increase in the generated total funds available, 1 denotes no change. this can increase or decrease the total funds available beyond the specified range. 
	   */
	  public Wallet(double minRange, double maxRange, int intersectionsInTrip, double surplusFactor) {
		  rand = new Random();
		  totalFundsAvailable = randomRange(minRange,maxRange);
		  bid = totalFundsAvailable / intersectionsInTrip;
		  totalFundsAvailable = totalFundsAvailable  * surplusFactor;
		  this.intersectionsInTrip = intersectionsInTrip;
	  }
	  /**
	   * Generates a wallet with a specified set of parameters.
	   * @param initialBid Sets the initial bid and becomes the default bid if the wallet is used over multiple intersections
	   * @param initialFunds Sets the total amount stored in this wallet
	   * @param intersectionsInTrip Sets the number of intersections expected to use this wallet
	   */
	  public Wallet(double initialBid, double initialFunds, int intersectionsInTrip){
		  bid = initialBid;
		  totalFundsAvailable = initialFunds;
		  this.intersectionsInTrip = intersectionsInTrip;
	  }
	  
	/**
	* @author Alexander Humphry
	* @return the bid of the current vehicle
	*/
	public double getCurrentBid() {
		return bid;
	}
	
	/**
	 * returns the current funds in the wallet
	 * @return total funds in wallet
	 */
	public double getRemainingFunds(){
		return totalFundsAvailable;
	}
	/**
	 * Sets a new default bid amount and can be set before each intersection.
	 * @param amount The new default bid
	 * @param complex Whether the new default bid is constrained by the remaining total funds available
	 * @return
	 */
	public double setBid(double amount, boolean complex) {
		if(complex){
			if(amount > totalFundsAvailable){
				bid = totalFundsAvailable;
			} else {
				bid = amount;
			}
			return bid;
		} else {
			return bid;
		}
	}
	/**
	 * Sets the new total wallet balance.
	 * @param amount the new wallet balance to be entered
	 */
	public void setWalletAmount(double amount){
		this.totalFundsAvailable = amount;
	}
	/**
	 * Subtracts current bid from funds, calculates new bid to store if the current bid reduces the wallets funds to a negative value.
	 * @return The total remaining funds after accepting the new bid
	 */
	public double processBid() {
		if(intersectionsInTrip > 0){
			if(bid > totalFundsAvailable){
				bid = totalFundsAvailable;
			}
		} else {
			bid = 0;
		}
		totalFundsAvailable -= bid;
		intersectionsInTrip--;
		return totalFundsAvailable;
	}
	/**
	 * Subtracts current bid from funds, calculates new bid to store if the current bid reduces the wallets funds to a negative value.
	 * @param modifier modifies the calculation of a new bid before subtracting from the wallet funds
	 * @return The total remaining funds after accepting the new bid
	 */
	public double processBid(double modifier) {
		if(intersectionsInTrip > 0){
			bid = bid * modifier;
			if(bid > totalFundsAvailable){
				bid = totalFundsAvailable;
			}
		} else {
			bid = 0;
		}
		totalFundsAvailable -= bid;
		intersectionsInTrip--;
		return totalFundsAvailable;
	}
	  private double randomRange(double min, double max){
		  Double minStore = min*100;
		  Double maxStore = max*100;
		  int minInt = minStore.intValue();
		  int maxInt = maxStore.intValue();
		  int resultToConvert = rand.nextInt((maxInt - minInt) + 1) + minInt;
		  return resultToConvert/100.0;
	  }
}
