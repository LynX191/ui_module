#include "mode_tomo_vm.h"

// My include
#include "../../../Script/Config/cfg_app.h"
// end

ModeTomoView::ModeTomoView(QObject* parent) : QObject{parent}
{
	_model = new ModeModel();
}

ModeTomoView::~ModeTomoView()
{
	delete _model;
}

ModeModel* ModeTomoView::model()
{
	return _model;
}

void ModeTomoView::setModel(QVector<ModeFeature>& models)
{
	_model->createData(models);
}

void ModeTomoView::setMode(std::string cmd, int value)
{
	Q_EMIT applyMode(QString::fromStdString(cmd), value);
}

void ModeTomoView::setModel(ModeModel* value)
{
	if(_model != value) {
		_model = value;
		Q_EMIT modelChanged();
	}
}

void ModeTomoView::modeUserChoiceSlots(QString config)
{
	Q_EMIT currentListModeChanged(config.toStdString());
}
