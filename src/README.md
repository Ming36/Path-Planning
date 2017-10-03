# Path Planing

The project below is part of Udacity's Self Driving Car Degree. The goals for the project are:

* Drive at least 4.32 miles
* Do not exceed speed limit
* Do not exceed acceleration and jerk limits
* Do not cause collisions
* Change lane of three seconds maximum
* Be able to change lanes

## Car Movement on Simulator

The first part consists on defining a path to the car. Frenet coordinates is an
essential part of this task. From Frenet coordinates we define the desired path
in X and Y coordinates to send to simulator. The desired position is based on the
desired lane we want our car to stay. This part of the project has been borrowed
from Udacity's walkthrough.

Small changes have been made in order to improve the speed control of the car,
a function called `speed_controller` has been created to act as a on/off controller.

This mechanism could be improved with a linear function limited to car's constraints.

## Path Generation
After the car is able to drive at desired lane, it's time to define which lane
to choose based on environment.

### Prediction
Path generation starts with prediction, by analyzing linearly where all other
cars will be for the next 2 seconds, i.e, for every message received from
simulator we evaluate the next 10 positions spaced 0.2 seconds between each other.

The collected data for each car is inserted to a vector of struct called pred.
Each pred contains the id of the car and its 10 predictions of Frenet coordinates.

```
vector<prediction> pred;
for(auto & t: traffic) {
  prediction p;
  p.id = t.id;
  p.predictions = t.Predict();
  pred.push_back(p);
}
```

### Get Next Lane
As part the program flow the ego object calls the `get_next_lane` function to determine:

1. if ego car will collide
2. determine next lane in case of collision is predicted

Collision is calculate if within the next 10 steps of prediction the car ahead will
be less than 15 meters distant from ego car, is so this function returns true and
the best next lane is calculated at `_get_safe_lane` function.

### Get Safe Lane
Safe Lane is determined when prediction over all other cars determines that no collisions
will occur with cars 15 meters ahead nor 10 meters behind. In case of two lanes
being evaluated as good candidates, the lower lane number is returned.

### Lane Change Buffer
To avoid rapid lane changes and reduce collision probability, a buffer has been created.
This buffer of limited size is populated every time that `Get Safe Lane` is called.
Once all positions of this buffer represents the same lane, so the car change its lane.

Lane number is then inserted to Path Generation function to change car's lane.

## Conclusion
Although this algorithm is very simple it has the ability to fulfill the project
requirements. Many improvements can be done, like creating a cost function and
determining best path to desired position, implementing a PID to keep at a constant
distance from other cars.
