#ifndef _MESH_VIEW_VIEWER_MAIN_WINDOW_
#define _MESH_VIEW_VIEWER_MAIN_WINDOW_

#include <string>
#include <vector>

#include <QObject>
#include <QStringListModel>
#include <QSignalMapper>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include "fab/geometry/template/customizer.h"
#include "fab/geometry/template/editor.h"
#include "fab/geometry/template/template.h"
#include "ui/ui_gen-inl.h"
#include "template_model_qt.h"

namespace mds {

class ViewerMainWindow : public QObject, private Ui::MainWindow {
  Q_OBJECT

 public:
  ViewerMainWindow();
  virtual ~ViewerMainWindow();

  void setupUi(QMainWindow *main_window);
  void setTemplateFromFileOrDie(const string& filename);
  void setTemplate(boost::shared_ptr<mit_plato::FabGeometryTemplate> tpl,
                   const string& filename = "");

 public Q_SLOTS:
  void deselectVariables();
  void updateGeometry();
  void refreshGeometry();
  void reopenFile();

  // TODO: use QSettings to save last used directory
  void newTemplate();
  void openTemplate();
  void saveTemplateAscii();
  void saveTemplateAsAscii();
  void saveMesh();

  // Editing (Shapes)
  void createShape(QString name);
  void modifyShape(QString name);
  void combineShapes(QString name);
  void deleteShape();

  // Editing (Constraints)
  void addConstraint();  // launches dialog
  void deleteConstraint();
  void editConstraint();
  void runSolver();
  void checkConstraints();
  void addDefaultChecks();
  void evaluateChecks();

  // Editing (Variables)
  void addMetaVar();
  void deleteMetaVar();
  void setMutable();
  void setImmutable();
  void controlClicked(QListWidgetItem* item);

  // Family
  void launchTestUI();

  // Selection
  void updateModelSelection(const QModelIndex&);
  void updateTreeViewSelection();

 Q_SIGNALS:
  void geometryUpdated();

 private:
  void geometryUpdateDone(bool success, mit_plato::TemplateCustomizer* c);

  void GetSelectedParams(vector<mit_plato::VarHandle>* handles) const;

  // Resets the template model using template stored in the editor
  // (Note: a shortcut to avoid implementing model editing operations)
  void resetModel();

  // Resets the string model used to display constraints
  void resetConstraintsModel();
  void resetMetavarModel();
  void resetControlsModel();
  void resetChecksModel();

  void setupSignals();
  void setupProgrammaticUi(QMainWindow *main_window);

  QMenu* createTopLevelMenu(const string& name);

  void AddMappedActionsToMenu(const std::set<string>& action_names,
                              QMenu* menu,
                              QSignalMapper* mapper);

  //  MeshViewToolbox toolbox;
  boost::scoped_ptr<mit_plato::TemplateCustomizer> customizer_;
  boost::scoped_ptr<mit_plato::TemplateEditor> editor_;
  boost::scoped_ptr<FabTemplateTreeModel> model_;
  boost::scoped_ptr<TemplateViewsController> views_controller_;
  QMainWindow* main_window_;
  std::string current_file_;

  // UI elements added programmatically
  std::vector<QAction*> actions_;

  // Operations
  boost::scoped_ptr<QMenu> menuOps_;
  boost::scoped_ptr<QMenu> submenuCreate_;
  boost::scoped_ptr<QMenu> submenuModify_;
  boost::scoped_ptr<QMenu> submenuCombine_;

  boost::scoped_ptr<QSignalMapper> createShapeMapper_;
  boost::scoped_ptr<QSignalMapper> modifyShapeMapper_;
  boost::scoped_ptr<QSignalMapper> combineShapeMapper_;

  // Constraints
  boost::scoped_ptr<QMenu> menuConstraints_;
  boost::scoped_ptr<QMenu> menuVars_;
  boost::scoped_ptr<QMenu> menuFamily_;

  // Models
  boost::scoped_ptr<QStringListModel> constraints_model_;
  std::vector<mit_plato::VarHandle> metavar_handles_;

  boost::mutex lock_;
  bool geometry_update_pending_;
  bool geometry_update_running_;
};

}  // namespace mds

#endif
