#include <QElapsedTimer>
#include <QTimer>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <OgreGpuProgramManager.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreMaterialManager.h>
#include <std_srvs/Empty.h>

#ifdef Q_OS_MAC
#include <ApplicationServices/ApplicationServices.h>
// Apparently OSX #defines 'check' to be an empty string somewhere.
// That was fun to figure out.
#undef check
#endif

#include <ros/console.h>
#include <ros/ros.h>

#include <rviz/env_config.h>
#include <rviz/ogre_helpers/ogre_logging.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/wait_for_master_dialog.h>

#include "tomo_rviz/visualizer_app.h"

#define CATCH_EXCEPTIONS 0

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace rviz
{
	bool reloadShaders(std_srvs::Empty::Request& /*unused*/, std_srvs::Empty::Response& /*unused*/)
	{
		ROS_INFO("[VisualizerApp] Reloading materials.");
		{
			Ogre::ResourceManager::ResourceMapIterator it = Ogre::MaterialManager::getSingleton().getResourceIterator();
			while(it.hasMoreElements()) {
				Ogre::ResourcePtr resource = it.getNext();
				resource->reload();
			}
		}
		ROS_INFO("[VisualizerApp] Reloading high-level gpu shaders.");
		{
			Ogre::ResourceManager::ResourceMapIterator it = Ogre::HighLevelGpuProgramManager::getSingleton().getResourceIterator();
			while(it.hasMoreElements()) {
				Ogre::ResourcePtr resource = it.getNext();
				resource->reload();
			}
		}
		ROS_INFO("[VisualizerApp] Reloading gpu shaders.");
		{
			Ogre::ResourceManager::ResourceMapIterator it = Ogre::GpuProgramManager::getSingleton().getResourceIterator();
			while(it.hasMoreElements()) {
				Ogre::ResourcePtr resource = it.getNext();
				resource->reload();
			}
		}
		return true;
	}

	VisualizerApp::VisualizerApp(CfgApp& cfg, QObject* parent)
		: config(cfg), QObject(parent), app_(nullptr), continue_timer_(nullptr), frame_(nullptr)
	{
		isCtrl	= false;
		isShift = false;
	}

	// Destructor.
	VisualizerApp::~VisualizerApp()
	{
		delete continue_timer_;
		delete frame_;
	}

	void VisualizerApp::setApp(QApplication* app)
	{
		app_ = app;
		app_->installEventFilter(this);
	}

	bool VisualizerApp::init(int argc, char* argv[])
	{
		ROS_INFO("rviz version %s", get_version().c_str());
		ROS_INFO("compiled against Qt version %s", QT_VERSION_STR);
		ROS_INFO("compiled against OGRE version %d.%d.%d%s (%s)", OGRE_VERSION_MAJOR, OGRE_VERSION_MINOR, OGRE_VERSION_PATCH,
				 OGRE_VERSION_SUFFIX, OGRE_VERSION_NAME);

#ifdef Q_OS_MAC
		ProcessSerialNumber PSN;
		GetCurrentProcess(&PSN);
		TransformProcessType(&PSN, kProcessTransformToForegroundApplication);
		SetFrontProcess(&PSN);
#endif

#if CATCH_EXCEPTIONS
		try {
#endif
			startContinueChecker();

			std::string display_config, fixed_frame, splash_path, help_path;
			int force_gl_version = 0;

			po::options_description options;
			options.add_options()  // clang-format off
      ("help,h", "Produce this help message")
      ("splash-screen,s", po::value<std::string>(&splash_path),
       "A custom splash-screen image to display")
      ("help-file", po::value<std::string>(&help_path),
       "A custom html file to show as the help screen")
      ("display-config,d", po::value<std::string>(&display_config),
       "A display config file (.rviz) to load")
      ("fullscreen", "Trigger fullscreen display")
      ("fixed-frame,f", po::value<std::string>(&fixed_frame), "Set the fixed frame")
      ("ogre-log,l", "Enable the Ogre.log file (output in cwd) and console output.")
      ("in-mc-wrapper", "Signal that this is running inside a master-chooser wrapper")
      ("opengl", po::value<int>(&force_gl_version),
        "Force OpenGL version (use '--opengl 210' for OpenGL 2.1 compatibility mode)")
      ("disable-anti-aliasing", "Prevent rviz from trying to use anti-aliasing when rendering.")
      ("no-stereo", "Disable the use of stereo rendering.")("verbose,v", "Enable debug visualizations")
      ("log-level-debug", "Sets the ROS logger level to debug.");
			// clang-format on
			po::variables_map vm;
			try {
				po::store(po::parse_command_line(argc, argv, options), vm);
				po::notify(vm);

				if(vm.count("help")) {
					std::cout << "rviz command line options:\n" << options;
					return false;
				}

				if(vm.count("log-level-debug")) {
					if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
						ros::console::notifyLoggerLevelsChanged();
					}
				}
			}
			catch(std::exception& e) {
				ROS_ERROR("Error parsing command line: %s", e.what());
				return false;
			}

			if(!ros::master::check()) {
				WaitForMasterDialog dialog;
				if(dialog.exec() != QDialog::Accepted) {
					return false;
				}
			}

			nh_.reset(new ros::NodeHandle);

			if(vm.count("ogre-log"))
				OgreLogging::useRosLog();

			RenderSystem::forceGlVersion(force_gl_version);

			if(vm.count("disable-anti-aliasing"))
				RenderSystem::disableAntiAliasing();

			if(vm.count("no-stereo"))
				RenderSystem::forceNoStereo();

			frame_ = new VisualizationFrame();
			frame_->setApp(app_);

			if(!help_path.empty()) {
				frame_->setHelpPath(QString::fromStdString(help_path));
			}

			frame_->setShowChooseNewMaster(vm.count("in-mc-wrapper") > 0);

			if(vm.count("splash-screen"))
				frame_->setSplashPath(QString::fromStdString(splash_path));

			frame_->initialize(QString::fromStdString(display_config));

			if(!fixed_frame.empty())
				frame_->getManager()->setFixedFrame(QString::fromStdString(fixed_frame));

			frame_->getManager()->getSelectionManager()->setDebugMode(vm.count("verbose") > 0);

			is_maximized = false;
			setRVizWindowState(true);
			// frame_->setWindowFlags(Qt::ToolTip |Qt::FramelessWindowHint  | Qt::WindowStaysOnTopHint);
			// frame_->setWindowState(Qt::WindowFullScreen);

			ros::NodeHandle private_nh("~");
			reload_shaders_service_ = private_nh.advertiseService("reload_shaders", reloadShaders);
			load_config_service_	= private_nh.advertiseService("load_config", &VisualizerApp::loadConfigCallback, this);
			save_config_service_	= private_nh.advertiseService("save_config", &VisualizerApp::saveConfigCallback, this);

#if CATCH_EXCEPTIONS
		}
		catch(std::exception& e) {
			ROS_ERROR("Caught exception while loading: %s", e.what());
			return false;
		}
#endif
		return true;
	}

	bool VisualizerApp::closeApp()
	{
		frame_->close();
	}

	void VisualizerApp::startContinueChecker()
	{
		continue_timer_ = new QTimer(this);
		connect(continue_timer_, SIGNAL(timeout()), this, SLOT(checkContinue()));
		continue_timer_->start(100);
	}

	void VisualizerApp::checkContinue()
	{
		if(!ros::ok()) {
			if(frame_) {
				// Make sure the window doesn't ask if we want to save first.
				frame_->setWindowModified(false);
			}
			QApplication::closeAllWindows();
		}
	}

	bool VisualizerApp::loadConfigCallback(SendFilePathRequest& req, SendFilePathResponse& res)
	{
		fs::path path = req.path.data;
		if(fs::is_regular_file(path))
			res.success = frame_->loadDisplayConfigHelper(path.string());
		else
			res.success = false;
		return true;
	}

	bool VisualizerApp::saveConfigCallback(SendFilePathRequest& req, SendFilePathResponse& res)
	{
		res.success = frame_->saveDisplayConfig(QString::fromStdString(req.path.data));
		return true;
	}

	void VisualizerApp::setRVizWindowState(bool active)
	{
		frame_->setWindowFlags(active ? (Qt::ToolTip | Qt::WindowStaysOnTopHint) : (Qt::ToolTip));
	}

	void VisualizerApp::setRVizMetric(bool isVisible, int x, int y, int width, int height, bool isMaximized)
	{
		_isVisible = isVisible;
		if(isVisible) {
			frame_->showPlugin(isMaximized);
			frame_->show();
			frame_->setGeometry(x, y, width, height);
			is_maximized = isMaximized;
		}
		else
			frame_->hide();
	}

	void VisualizerApp::setRvizVisible(bool value)
	{
		if(value)
			frame_->show();
		else
			frame_->hide();
	}
	void VisualizerApp::setRVizPanelDisplay(bool isRvizFullScreen)
	{
		if(isRvizFullScreen) {
			// config.writeFileConfig();
			this->setRVizMetric(true, 0, 0, 1920, 1080, true);
			frame_->showPlugin(true);
		}
		else {
			setRVizMetric(true, xRviz, yRviz, widthRviz, heightRviz, false);
			frame_->showPlugin(false);
		}
	}
}  // namespace rviz
