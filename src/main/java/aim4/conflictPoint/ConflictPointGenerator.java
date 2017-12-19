package aim4.conflictPoint;

import java.awt.geom.Point2D;
import java.util.List;

import aim4.im.Intersection;
import aim4.im.IntersectionManager;

public interface ConflictPointGenerator {

	public List<Point2D> generateConflictPoints(Intersection i, boolean restrictedTurning);
}
