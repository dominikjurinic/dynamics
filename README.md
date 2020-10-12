# Numerical Methods of Dynamics of Structural Systems
Source code for dynamics simulation of  Mars's rover rocker bogey mechanism.

## Model
CAD Model was built using Solidworks (see `Rover.step`), coupled with project Chrono architecture (https://projectchrono.org/). Goal was to obtain qualitatively similar results to experimental data (Arvidson, R. E et al.).

## Results
Relevant papers and sources used for model validation:
- Arvidson, R. E., DeGrosse, P., Jr., Grotzinger, J. P., Heverly, M. C., Shechet, J.,
Moreland, S. J., Newby, M. A., Stein, N., Steffy, A. C., Zhou, F., Zastrow, A. M.,
Vasavada, A. R., Fraeman, A. A., Stilly, E. K. Relating geologic units and mobility
system kinematics contributing to Curiosity wheel damage at Gale Crater, Mars. //
Journal of Terramechanics. Vol. 73, 2017, str. 73-93.
- https://ntrs.nasa.gov/search.jsp?R=19900007837


Experimental data:

![experimentalResult](https://user-images.githubusercontent.com/58303666/95742190-e7939e80-0c8f-11eb-8eee-70b7c2096769.PNG)


Simulation data:

![numericalResult](https://user-images.githubusercontent.com/58303666/95742311-1578e300-0c90-11eb-8587-9336198d9c88.PNG)


## Note
Resultant forces in absolute values ​​are not exact, as uniformly distributed mass density is assumed, which is not case in real rover, but  decent qualitative result match is achieved.
For more results, please, contact me.
