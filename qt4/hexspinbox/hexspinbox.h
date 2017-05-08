#ifndef HEX_SPIN_BOX_H
#define HEX_SPIN_BOX_H

#include <QSpinBox>
class QRegExpValidator;

class HexSpinBox : public QSpinBox
{
	Q_OBJECT
public:
	HexSpinBox(QWidget * parent = 0);
protected:
	QValidator::State validate(QString& text, int &pos) const;
	int valueFromText(const QString& text) const;
	QString textFromValue(int value) const;
private:
	QRegExpValidator* validator;
};


#endif
