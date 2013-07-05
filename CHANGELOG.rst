^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package octomap_rviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'groovy-devel' into hydro-devel
* Merge pull request `#5 <https://github.com/OctoMap/octomap_rviz_plugins/issues/5>`_ from OctoMap/groovy-devel
  Porting fixes from groovy branch (QT4_WRAP macro, OCTOMAP_INCLUDE_DIRS)

0.0.4 (2013-07-05)
------------------
* Safer checking for octree conversion from stream
* Merge pull request `#6 <https://github.com/OctoMap/octomap_rviz_plugins/issues/6>`_ from sotte/groovy-devel-fix-issue1
  Create octomap using AbstracOcTree.
* Create octomap using the AbstracOcTree.
  This is implemented as suggested in issue `#1 <https://github.com/OctoMap/octomap_rviz_plugins/issues/1>`_ and also fixes issue `#1 <https://github.com/OctoMap/octomap_rviz_plugins/issues/1>`_.

0.0.3 (2013-06-26)
------------------
* Version increased to 0.1.0
* Merge pull request `#3 <https://github.com/OctoMap/octomap_rviz_plugins/issues/3>`_ from jihoonl/hydro-devel
  correct CMakeLists. octomap_INCLUDE_DIRS(or LIB..) to OCTOMAP_INCLUDE_DI...
* correct CMakeLists. octomap_INCLUDE_DIRS(or LIB..) to OCTOMAP_INCLUDE_DIRS

0.0.2 (2013-05-08)
------------------
* 0.0.1 -> 0.0.2
* adding OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED option to make QT4_WRAP macro happy

0.0.1 (2013-05-04)
------------------
* removing dependency to octomap_ros package + clean-up
* removing rosbuild Makefile
* major revision
* adjusting to recent rviz api changes
* major update
* removed pcl dependancy
* more thread-safety..
* minor
* fixed threading issue
* worked on mutex/threading issue
* minor
* working version with ogre point cloud structures and colored boxes
* switched to ogre pointcloud display
* minor
* minor
* added max depth filter
* minor
* first working version with voxel display
* first running version
* fixed "subscribing when deactivated" bug
* initial commit
* first commit
