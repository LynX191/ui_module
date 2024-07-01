#ifndef STATETOMOVM_H
#define STATETOMOVM_H

#include <QObject>
#include <QVector>

// My include
#include "../../../Model/TomoModel/state_model.h"
// end

class StateTomoView : public QObject
{
	Q_OBJECT
    Q_PROPERTY(StateModel* model READ model WRITE setModel NOTIFY modelChanged)

public:
	explicit StateTomoView(QObject* parent = nullptr);
	~StateTomoView();

	StateModel* model();
    void setModel(QVector<StateFeature>& models);

private:
	StateModel* _model;

Q_SIGNALS:
	void modelChanged();
    void currentListStateChanged(std::string);

public Q_SLOTS:
	void setModel(StateModel* value);
    void stateUserChoiceSlots(int stateId);
};

#endif	// STATETOMOVM_H
