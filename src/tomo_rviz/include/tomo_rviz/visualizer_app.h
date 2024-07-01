#ifndef VISUALIZER_APP_H
#define VISUALIZER_APP_H

#include <QApplication>
#include <QObject>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/SendFilePath.h>

#endif

#include "../tomo_ui/src/Script/Config/cfg_app.h"
// #include "../tomo_ui/src/ViewModel/master_app.h"

#include "tomo_rviz/visualization_frame.h"

namespace rviz
{

	class VisualizerApp : public QObject
	{
		Q_OBJECT
	public:
		explicit VisualizerApp(CfgApp& cfg, QObject* parent = nullptr);
		~VisualizerApp();

		void setApp(QApplication* app);
		bool init(int argc, char* argv[]);

		// Iric implement
		bool closeApp();
		// end

	protected:
		// bool eventFilter(QObject* obj, QEvent* event) override;

	private Q_SLOTS:
		void checkContinue();

	private:
		void startContinueChecker();

		bool loadConfigCallback(SendFilePathRequest& req, SendFilePathResponse& res);
		bool saveConfigCallback(SendFilePathRequest& req, SendFilePathResponse& res);

		CfgApp& config;

		// Iric implement
		bool isCtrl;
		bool isShift;
		bool is_maximized;
		bool _isVisible = true;
		// end

		QApplication* app_;
		QTimer* continue_timer_;
		VisualizationFrame* frame_;
		ros::NodeHandlePtr nh_;
		ros::ServiceServer reload_shaders_service_;
		ros::ServiceServer load_config_service_;
		ros::ServiceServer save_config_service_;

		int xRviz;
		int yRviz;
		int heightRviz;
		int widthRviz;

	public Q_SLOTS:
		// Iric implement
		void setRVizPanelDisplay(bool isRvizFullScreen);
		void setRVizMetric(bool, int, int, int, int, bool);
		void setRvizVisible(bool);
		void setRVizWindowState(bool);
		// end

	Q_SIGNALS:
		// Iric implement
		void initRvizWindow();
		// end
	};
}  // namespace rviz
#endif	// VISUALIZER_APP_H
