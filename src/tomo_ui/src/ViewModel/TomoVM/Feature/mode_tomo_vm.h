#ifndef MODETOMOVM_H
#define MODETOMOVM_H

#include <QObject>
#include <QVector>

// My include
#include "../../../Model/TomoModel/mode_model.h"
// end

class ModeTomoView : public QObject
{
	Q_OBJECT
	Q_PROPERTY(ModeModel* model READ model WRITE setModel NOTIFY modelChanged)

public:
	explicit ModeTomoView(QObject* parent = nullptr);
	~ModeTomoView();

	ModeModel* model();
	void setModel(QVector<ModeFeature>& models);
	void setMode(std::string, int);

private:
	ModeModel* _model;

Q_SIGNALS:
	void modelChanged();
	void applyMode(QString acmd, int avalue);
	void currentListModeChanged(std::string);

public Q_SLOTS:
	void setModel(ModeModel* value);
	void modeUserChoiceSlots(QString);
};

#endif	// MODETOMOVM_H
