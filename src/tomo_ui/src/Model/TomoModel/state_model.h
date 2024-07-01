#ifndef STATEMODEL_H
#define STATEMODEL_H

#include <QAbstractListModel>

struct StateFeature{
    QString name;
    bool isCheck;
};

class StateModel: public QAbstractListModel
{
    Q_OBJECT

public:
    explicit StateModel(QObject *parent = nullptr);
    ~StateModel();

    enum DataRoles {
        nameRole = Qt::UserRole,
        isCheckRole = Qt::UserRole+1
    };
    // Basic functionality:
    void addData(const StateFeature& entry);
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override;

    // Editable:
    void editInputData(int row, const QVariant &value, int role = Qt::EditRole);
    void createData(QVector<StateFeature> data);


    QVector<StateFeature> _model;
};

#endif // STATEMODEL_H
