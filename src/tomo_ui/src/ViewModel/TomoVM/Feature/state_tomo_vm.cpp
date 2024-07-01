#include "state_tomo_vm.h"

// My include
#include "../../../Script/Config/cfg_app.h"
// end

StateTomoView::StateTomoView(QObject* parent) : QObject{parent}
{
	_model = new StateModel();
}

StateTomoView::~StateTomoView()
{
	delete _model;
}

StateModel* StateTomoView::model()
{
	return _model;
}

void StateTomoView::setModel(QVector<StateFeature>& models)
{
	_model->createData(models);
}

void StateTomoView::setModel(StateModel* value)
{
	if(_model != value) {
		_model = value;
		Q_EMIT modelChanged();
	}
}

void StateTomoView::stateUserChoiceSlots(int stateId)
{
	Q_EMIT currentListStateChanged(std::to_string(stateId));
}
