#ifndef VISUALIZATION_FRAME_H
#define VISUALIZATION_FRAME_H

#include <boost/shared_ptr.hpp>

#include <QList>
#include <QMainWindow>

#include <deque>
#include <string>

#include <rviz/config.h>
#include <rviz/panel.h>
#include <rviz/rviz_export.h>
#include <rviz/window_manager_interface.h>

// #include <rviz/panel_factory.h>
// #include <rviz/preferences.h>
// #include <rviz/tool.h>
// #include <rviz/widget_geometry_change_detector.h>

#include <ros/time.h>

class QSplashScreen;
class QAction;
class QActionGroup;
class QTimer;
class QDockWidget;
class QLabel;
class QToolButton;

namespace rviz
{
	class PanelFactory;
	struct Preferences;
	class RenderPanel;
	class VisualizationManager;
	class Tool;
	class WidgetGeometryChangeDetector;

	class VisualizationFrame : public QMainWindow, public WindowManagerInterface
	{
		Q_OBJECT
	public:
		VisualizationFrame(QWidget* parent = nullptr);
		~VisualizationFrame();

		void setApp(QApplication* app);

		void setShowChooseNewMaster(bool show);

		void setHelpPath(const QString& help_path);

		void setSplashPath(const QString& splash_path);

		void initialize(const QString& display_config_file = "");

		VisualizationManager* getManager()
		{
			return manager_;
		}

		// overrides from WindowManagerInterface
		QWidget* getParentWindow();
		PanelDockWidget* addPane(const QString& name, QWidget* panel, Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
								 bool floating = false);

		QDockWidget* addPanelByName(const QString& name, const QString& class_lookup_name, Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
									bool floating = false);

		void loadPersistentSettings();

		void savePersistentSettings();

		void loadDisplayConfig(const QString& path);

		bool loadDisplayConfigHelper(const std::string& full_path);

		bool saveDisplayConfig(const QString& path);

		QString getErrorMessage() const
		{
			return error_message_;
		}

		virtual void load(const Config& config);

		virtual void save(Config config);

		void setHideButtonVisibility(bool visible);

		// Iric implement
		bool is_close_done;
		void showPlugin(bool show);
		// end

	private:
		// Iric implement

		std::shared_ptr<PanelDockWidget> motion_planing_pdw_ptr;
		std::shared_ptr<PanelDockWidget> display_panel_pdw_ptr;

		bool display_panel_status;
		bool motion_panel_status;
		QToolButton* display_button_ptr;
		QToolButton* motion_button_ptr;
		// end

	public Q_SLOTS:
		void setDisplayConfigModified();
		void setStatus(const QString& message);
		void setFullScreen(bool full_screen);
		void exitFullScreen();

		// Iric implement
		void toolPluginChanged(bool&);
		// end

	Q_SIGNALS:
		void statusUpdate(const QString& message);
		void fullScreenChange(bool hidden);

		void displayConfigFileChanged(const QString& fullpath);
		// Iric implement
		// end

	protected Q_SLOTS:
		void onOpen();
		void onSave();
		void onSaveAs();
		void onSaveImage();
		void onRecentConfigSelected();
		void onHelpWiki();
		void onHelpAbout();
		void openNewPanelDialog();
		void openNewToolDialog();
		void openPreferencesDialog();
		void showHelpPanel();
		void onDockPanelChange();

		void onToolbarRemoveTool(QAction* remove_tool_menu_action);

		void onButtonStyleTool(QAction* button_style_tool_menu_action);

		void onToolbarActionTriggered(QAction* action);

		void addTool(Tool* tool);

		void onToolNameChanged(const QString& name);

		void removeTool(Tool* tool);

		void refreshTool(Tool* tool);

		void indicateToolIsCurrent(Tool* tool);

		void changeMaster();

		void onDeletePanel();

	protected Q_SLOTS:
		void markLoadingDone();

		void setImageSaveDirectory(const QString& directory);

		void reset();

		void onPanelDeleted(QObject* dock);
		void onHelpDestroyed();

		void hideLeftDock(bool hide);
		void hideRightDock(bool hide);

		virtual void onDockPanelVisibilityChange(bool visible);

		void updateFps();

		// Some slots Iric implement
		// end

	protected:
		void initConfigs();

		void initMenus();

		void initToolbars();

		bool prepareToExit();

		void closeEvent(QCloseEvent* event) override;

		void leaveEvent(QEvent* event) override;

		void markRecentConfig(const std::string& path);
		void updateRecentConfigMenu();

		void loadPanels(const Config& config);

		void configureToolbars(const Config& config);

		void saveToolbars(Config config);

		void savePanels(Config config);

		void loadPreferences(const Config& config);
		void savePreferences(Config config);

		void loadWindowGeometry(const Config& config);

		void saveWindowGeometry(Config config);

		void setDisplayConfigFile(const std::string& path);

		void hideDockImpl(Qt::DockWidgetArea area, bool hide);

		QApplication* app_;

		RenderPanel* render_panel_;

		QAction* show_help_action_;

		std::string config_dir_;
		std::string persistent_settings_file_;
		std::string display_config_file_;
		std::string default_display_config_file_;
		std::string last_config_dir_;
		std::string last_image_dir_;
		std::string home_dir_;

		boost::shared_ptr<Preferences> preferences_;

		QMenu* file_menu_;
		QMenu* recent_configs_menu_;
		QMenu* view_menu_;
		QMenu* delete_view_menu_;
		QMenu* plugins_menu_;

		QToolBar* toolbar_;

		VisualizationManager* manager_;

		std::string package_path_;
		QString help_path_;
		QString splash_path_;

		QSplashScreen* splash_;

		typedef std::deque<std::string> D_string;
		D_string recent_configs_;

		QActionGroup* toolbar_actions_;
		std::map<QAction*, Tool*> action_to_tool_map_;
		std::map<Tool*, QAction*> tool_to_action_map_;
		bool show_choose_new_master_option_;

		QToolButton* hide_left_dock_button_;
		QToolButton* hide_right_dock_button_;

		PanelFactory* panel_factory_;

		struct PanelRecord
		{
			Panel* panel;
			PanelDockWidget* dock;
			QString name;
			QString class_id;
			QAction* delete_action;
		};
		QList<PanelRecord> custom_panels_;

		QAction* add_tool_action_;
		QMenu* remove_tool_menu_;

		bool initialized_;
		WidgetGeometryChangeDetector* geom_change_detector_;
		bool loading_;			   ///< True just when loading a display config file, false all other times.
		QTimer* post_load_timer_;  ///< Single-shot timer for calling postLoad() a short time after
								   /// loadDisplayConfig() finishes.

		QLabel* status_label_;
		QLabel* fps_label_;
		QStatusBar* original_status_bar_;

		int frame_count_;
		ros::WallTime last_fps_calc_time_;

		QString error_message_;	 ///< Error message (if any) from most recent saveDisplayConfig() call.

		bool toolbar_visible_;
	};
}  // namespace rviz
#endif	// VISUALIZATION_FRAME_H
