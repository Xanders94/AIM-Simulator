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
	  
	  private static final double min = 0.05;
	  private static final double max = 0.50;
	  
	  public Wallet(){
		  rand = new Random();
		  totalFundsAvailable = randomRange(min,max);
		  bid = totalFundsAvailable;
		  intersectionsInTrip = 1;
		  
	  }
	  public Wallet(double initialBid, double initialFunds, int intersectionsInTrip){
		  bid = initialBid;
		  totalFundsAvailable = initialFunds;
		  this.intersectionsInTrip = intersectionsInTrip;
	  }
	  
	/**
	* @author Alex Humphry
	* return the bid of the current vehicle
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
	
	public void setWalletAmount(double amount){
		this.totalFundsAvailable = amount;
	}
	/**
	 * @author Alex Humphry
	 * 
	 * subtracts current bid from funds, calculates new bid to store
	 */
	/*public void acceptBid() {
		totalFundsAvailable -= bid;
		intersectionsInTrip -= 1;
		if(intersectionsInTrip > 0){
			bid = totalFundsAvailable / intersectionsInTrip;
			if(bid > totalFundsAvailable){
				bid = totalFundsAvailable;
			}
		} else {
			bid = 0;
		}
	}*/
	/**
	 * subtracts current bid from funds, calculates new bid to store
	 * @param modifier, % under or over new bid should be compared to default calculation
	 */
	/*public void acceptBid(double modifier) {
		totalFundsAvailable -= bid;
		if(intersectionsInTrip > 0){
			bid = (totalFundsAvailable / intersectionsInTrip) * (modifier/100);
			if(bid > totalFundsAvailable){
				bid = totalFundsAvailable;
			}
		} else {
			bid = 0;
		}
		intersectionsInTrip--;
	}*/
	  private double randomRange(double min, double max){
		  Double minStore = min*100;
		  Double maxStore = max*100;
		  int minInt = minStore.intValue();
		  int maxInt = maxStore.intValue();
		  int resultToConvert = rand.nextInt((maxInt - minInt) + 1) + minInt;
		  return resultToConvert/100.0;
	  }
}
