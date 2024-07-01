#ifndef RECOVERY_H
#define RECOVERY_H

#include <QObject>

class Recovery : public QObject
{
    Q_OBJECT
public:
    explicit Recovery(QObject *parent = nullptr);
    ~Recovery();
    std::string _name;
    void setName(char **argv);
private:
    void process();

signals:
    void popupModalDialog(bool isFunctional, int type, QString messageID, QString content);
public slots:
    void close();
};

#endif // RECOVERY_H
