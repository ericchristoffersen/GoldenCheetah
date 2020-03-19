/*
 * Copyright (c) 2020 Mark Liversedge (liversedge@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _GC_UserChart_h
#define _GC_UserChart_h 1

#include "GoldenCheetah.h"
#include "Settings.h"
#include "Context.h"
#include "Athlete.h"
#include "Colors.h"
#include "RCanvas.h"

#include "GenericSelectTool.h"
#include "GenericLegend.h"
#include "GenericChart.h"
#include "GenericPlot.h"
#include "ColorButton.h"

// the chart
class UserChartSettings;
class DataFilterEdit;
class UserChart : public GcChartWindow {

    Q_OBJECT

    // settings are saved as a json document and parsed using qt5 json support
    // we can rely on JSON support since we also need Qt Charts which is qt5 also
    Q_PROPERTY(QString settings READ settings WRITE applySettings USER true)

    public:

        UserChart(Context *context, bool rangemode);

        // for read and write of settings via chart properties
        QString settings() const;
        void applySettings(QString);

    public slots:

        // runtime - ride item changed
        void setRide(RideItem*);

        // runtime - date range was selected
        void setDateRange(DateRange);

        // redraw
        void refresh();

        // global config changed?
        void configChanged(qint32);

        // chart config changed?
        void chartConfigChanged();

    protected:

        // the actual config
        GenericChartInfo chartinfo;
        QList<GenericSeriesInfo> seriesinfo;
        QList<GenericAxisInfo> axisinfo;

    private:

        Context *context;
        bool rangemode;
        bool stale;
        RideItem *last; // the last ride we plotted

        RideItem *ride;
        DateRange dr;

        GenericChart *chart;
        UserChartSettings *settingsTool;
};

class UserChartSettings : public QWidget {

    Q_OBJECT

    public:
        UserChartSettings(Context *, bool rangemode, GenericChartInfo &, QList<GenericSeriesInfo> &, QList<GenericAxisInfo> &);

    private:
        Context *context;
        bool rangemode;

        // settings maintained here
        GenericChartInfo &chartinfo;
        QList<GenericSeriesInfo> &seriesinfo;
        QList<GenericAxisInfo> &axisinfo;

        // tabs
        QTabWidget *tabs;
        QWidget *chartpage;

        // series tab
        QTableWidget *seriesTable;
        QPushButton *editSeriesButton, *addSeriesButton, *deleteSeriesButton;
#ifndef Q_OS_MAC
        QToolButton *upSeriesButton, *downSeriesButton;
#else
        QPushButton *upSeriesButton, *downSeriesButton;
#endif

        // axes tab
        QTableWidget *axisTable;
        QPushButton *editAxisButton, *addAxisButton, *deleteAxisButton;

    public slots:

        // configuration - chart
        void refreshChartInfo(); // update gui with loaded config
        void updateChartInfo();

        // configuration - data series
        void refreshSeriesTab(); // update gui with current config
        void editSeries();
        void seriesClicked(int,int);
        void addSeries();
        void deleteSeries();

        // configuration - axes
        void refreshAxesTab(); // update gui with current config
        void editAxis();
        void axisClicked(int,int);
        void addAxis();
        void deleteAxis();

    signals:

        void chartConfigChanged();

    private:

        bool updating;

        // chart page
        QLineEdit *title;
        QTextEdit *description;
        QCheckBox *animate;
        QComboBox *type;
        QComboBox *legpos;
        QCheckBox *stack;
        QComboBox *orientation;
};

class EditUserSeriesDialog : public QDialog
{
    Q_OBJECT

    public:
        EditUserSeriesDialog(Context *, bool rangemode, GenericSeriesInfo &);

    public slots:
        void okClicked();
        void cancelClicked();

        void setErrors(QStringList &errors);

    private:
        Context *context;
        GenericSeriesInfo &original;

        // series page
        QLineEdit *name, *xname, *yname, *groupname;
        DataFilterEdit *program;
        QLabel *errors;

        QComboBox *line, *symbol;
        QDoubleSpinBox *size;
        ColorButton *color;
        QSpinBox *opacity;
        QCheckBox *opengl, *legend;
        QLineEdit *labels, *colors;

        QPushButton *okButton, *cancelButton;

};

class EditUserAxisDialog : public QDialog
{
    Q_OBJECT

    public:
        EditUserAxisDialog(Context *,GenericAxisInfo &);

    public slots:
        void okClicked();
        void cancelClicked();

    private:
        Context *context;
        GenericAxisInfo &original;

        // axes page
        QLineEdit *axisname;
        QComboBox *axistype;
        QCheckBox *log;
        QCheckBox *fixed;
        QDoubleSpinBox *min, *max;
        ColorButton *axiscolor;
        QLineEdit *categories;

        QPushButton *okButton, *cancelButton;
};
#endif
