^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rc_silhouettematch_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.1 (2019-05-14)
------------------
* add missing msg dependencies

3.0.0 (2019-05-13)
------------------
* refactoring and cleanup
* set parameters on parameter server so that dynamic reconfigure picks them up
* enum for dynamic reconfigure quality params
* service calls always return true, but set return_code in case of error
* visualize base plane as disk
* add plane_preference
* initial implementation of SilhouetteMatch client
* Contributors: Elena Gambaro, Felix Ruess, schaller
