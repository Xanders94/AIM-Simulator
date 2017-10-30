package aim4.conflictPoint;

import java.awt.geom.Point2D;
import java.util.List;

import aim4.im.IntersectionManager;

public interface ConflictPointGenerator {

	public List<Point2D> generateConflictPoints(IntersectionManager im, double pointMergeDist, boolean restrictedTurning);
}
