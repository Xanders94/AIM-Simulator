package aim4.conflictPoint;

import java.awt.geom.Point2D;
import java.util.List;

import aim4.im.Intersection;
import aim4.im.IntersectionManager;

public interface ConflictPointGenerator {
	/**
	 * Generates a set of conflict points for a given intersection. The given intersection is stored for future use
	 * @param i The intersection to be passed into the conflict point generator
	 * @param restrictedTurning Whether turning paths are allowed to cross adjacent lane paths
	 * @return a list of points within the intersection limits where vehicle paths from different lanes meet
	 */
	public List<Point2D> generateConflictPoints(Intersection i, boolean restrictedTurning);
}
