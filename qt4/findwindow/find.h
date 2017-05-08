#ifndef FIND_H
#define FIND_H

#include <QDialog>

class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;

class FindDialog : public QDialog
{
	Q_OBJECT
public:
	FindDialog(QWidget* parent = 0);
//singals:
//	void findNext(const QString &str, Qt::CaseSensitivity cs);
//	void findPrevious(const QString &str, Qt::CaseSensitivity cs);
private slots:
	void findClicked();
	void enableFindButton(const QString & text);
private:
	QLabel *m_plabel;
	QLineEdit *m_plineEdit;
	QCheckBox *m_pcheckBox;
	QCheckBox *m_pbackcheckBox;
	QPushButton *m_pfindButton;
	QPushButton *m_pcloseButton;
};

























#endif
