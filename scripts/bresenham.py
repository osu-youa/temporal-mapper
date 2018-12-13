#!/usr/bin/env python


from numpy import sign


def covered_cells(start, end):
    """
    Cells covered by a ray from the start cell to the end cell.  This implements Bresenham's algorithm.

    Arguments:
    start -- (x,y) position of the start cell
    end -- (x,y) position of the end cell
    """

    # We'll need the lengths of the ranges later on.
    x_range = end[0] - start[0]
    y_range = end[1] - start[1]

    # If the start and the end are the same point, then we don't do
    # anything.
    if x_range == 0 and y_range == 0:
        yield start
        raise StopIteration
    
    # Step through x or y?  Pick the one with the longer absolute
    # range.
    if abs(x_range) > abs(y_range):
        y_step = float(y_range) / abs(float(x_range))
        y = float(start[1])
        for x in xrange(start[0], end[0], sign(x_range)):
            yield((x, int(round(y))))
            y += y_step
    else:
        x_step = float(x_range) / abs(float(y_range))
        x = float(start[0])
        for y in xrange(start[1], end[1], sign(y_range)):
            yield((int(round(x)), y))
            x += x_step

    yield end

    raise StopIteration
