package aim4.driver;

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
	  public Wallet(){
		  bid = 0;
		  totalFundsAvailable = 0;
		  intersectionsInTrip = 0;
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
	
	public double setBid(double amount) {
		if(amount > totalFundsAvailable){
			bid = totalFundsAvailable;
		} else {
			bid = amount;
		}
		return bid;
	}
	
	public void setWalletAmount(double amount){
		
	}
	/**
	 * @author Alex Humphry
	 * 
	 * subtracts current bid from funds, calculates new bid to store
	 */
	public void acceptBid() {
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
	}
	/**
	 * subtracts current bid from funds, calculates new bid to store
	 * @param modifier, % under or over new bid should be compared to default calculation
	 */
	public void acceptBid(double modifier) {
		totalFundsAvailable -= bid;
		intersectionsInTrip -= 1;
		if(intersectionsInTrip > 0){
			bid = (totalFundsAvailable / intersectionsInTrip) * (modifier/100);
			if(bid > totalFundsAvailable){
				bid = totalFundsAvailable;
			}
		} else {
			bid = 0;
		}
	}
}
