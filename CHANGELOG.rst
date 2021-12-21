2.0.0 (2021-12-21)
------------------
* ROS2 Migration (`#41 <https://github.com/OctoMap/octomap_rviz_plugins/issues/41>`_)
* Contributors: Daisuke Nishimatsu, Wolfgang Merkt

0.2.4 (2021-12-21)
------------------
* Use automoc feature of CMake (`#34 <https://github.com/OctoMap/octomap_rviz_plugins/issues/34>`_)
* Contributors: Simon Schmeisser (isys vision), Wolfgang Merkt

0.2.3 (2021-01-17)
------------------
* Build plugin-lib as module (`#36 <https://github.com/OctoMap/octomap_rviz_plugins/issues/36>`_)
* Add GitHub Actions CI, address warnings for ROS Noetic (`#39 <https://github.com/OctoMap/octomap_rviz_plugins/issues/39>`_)
* Contributors: Sebastian Kasperski, Wolfgang Merkt

0.2.2 (2019-07-16)
------------------
* Add missing Qt5 dependencies `#35 <https://github.com/OctoMap/octomap_rviz_plugins/issues/35>`_
* Contributors: Wolfgang Merkt

0.2.1 (2019-07-11)
------------------
* Add Wolfgang Merkt to maintainers
* `Update visualization on property changes with latched topics <https://github.com/OctoMap/octomap_rviz_plugins/issues/32>`_
* `Update node frame from TF on every frame <https://github.com/OctoMap/octomap_rviz_plugins/issues/31>`_
* `Fixed rendering of free-space voxels and pruning enclosed nodes <https://github.com/OctoMap/octomap_rviz_plugins/issues/30>`_
* Contributors: Armin Hornung, Matthias Nieuwenhuisen, Vladimir Ivan, Wolfgang Merkt

0.2.0 (2016-07-10)
------------------
* Fix: RViz uses Qt5 in kinetic
* Contributors: Armin Hornung

0.1.0 (2016-07-07)
------------------
* Remove -ldefault_plugin from linker options, fixes `#19 <https://github.com/OctoMap/octomap_rviz_plugins/issues/19>`_
* Support for displaying ColorOcTree and OcTreeStamped, templated rviz plugins 
* Trim Z values in the octomap visualization
* Add alpha property to OccupancyGridDisplay
* add fix for qt moc BOOST_JOIN problem for osx yosemite build
* Contributors: Armin Hornung, Felix Endres, Gautham Manoharan, Javier V. Gómez, Oleksandr Lavrushchenko, Ryohei Ueda

0.0.5 (2013-09-06)
------------------
* fix a crash when the destructor is called before onInitialize
* Fix descriptions, limit octree depth range
* Porting fixes from groovy branch (QT4_WRAP macro, OCTOMAP_INCLUDE_DIRS)

0.0.4 (2013-07-05)
------------------
* Safer checking for octree conversion from stream
* Create octomap using AbstracOcTree (Fix issue #1)

0.0.3 (2013-06-26)
------------------
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
* removed pcl dependancy
* fixed threading issue
* working version with ogre point cloud structures and colored boxes
* added max depth filter
