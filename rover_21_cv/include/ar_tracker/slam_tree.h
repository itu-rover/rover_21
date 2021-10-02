#ifndef SLAM_TREE_H
#define SLAM_TREE_H

namespace ArTracker {
/*
        Main data structure of slam system
        - id: in aruco every marker in dictionary has uniqe id(>0) hence
        every marker can be identical we use arucos id for searching operation
        - name: object name for ros tf system
        - R: its rotation relative to "world"
        - T: its translation relative to "world"
        - left & right: pointers for sub trees
*/
struct slam_obj {
  int id;
  std::string name;

  tf::Quaternion R;
  tf::Vector3 T;

  slam_obj* left;
  slam_obj* right;
};

/*
        Interface class between system and data structure
*/
class slam_tree {
  slam_obj* root;
  slam_obj* add_(slam_obj*, slam_obj*);
  slam_obj* search_id_(slam_obj*, int);
  void traverse_(slam_obj*, tf::TransformBroadcaster, void (*action)(tf::TransformBroadcaster, slam_obj*));
  void delete_(slam_obj*);
  void deallocate(slam_obj*);

 public:
  slam_tree();
  void add(slam_obj*);
  void traverse(tf::TransformBroadcaster, void (*action)(tf::TransformBroadcaster, slam_obj*));
  void reset();
  slam_obj* search_id(int);
};

slam_tree::slam_tree() { root = NULL; }

/*
        bellow three functions are serve as interface
        between class and respective main opearion
        function
*/
void slam_tree::add(slam_obj* obj) { root = add_(root, obj); }

slam_obj* slam_tree::search_id(int id) { return search_id_(root, id); }

void slam_tree::traverse(tf::TransformBroadcaster br, void (*action)(tf::TransformBroadcaster br, slam_obj* obj)) {
  if (root != NULL) traverse_(root, br, action);
}

void slam_tree::reset() { delete_(root); }

/*
        bellow three functions are main binary tree operation functions
*/
slam_obj* slam_tree::add_(slam_obj* root, slam_obj* new_object) {
  if (root == NULL) return new_object;

  if (new_object->id > root->id)
    root->right = add_(root->right, new_object);

  else
    root->left = add_(root->left, new_object);

  return root;
}

slam_obj* slam_tree::search_id_(slam_obj* root, int id) {
  if (root == NULL) return root;

  if (root->id == id) return root;

  if (root->id > id)
    return search_id_(root->left, id);

  else
    return search_id_(root->right, id);
}

/*
        action function pointer determines what action to be taken by objects in tree
*/
void slam_tree::traverse_(slam_obj* root, tf::TransformBroadcaster br,
                          void (*action)(tf::TransformBroadcaster br, slam_obj* obj)) {
  if (root->right != NULL) traverse_(root->right, br, action);

  action(br, root);

  if (root->left != NULL) traverse_(root->left, br, action);
}

void slam_tree::delete_(slam_obj* obj) {
  if (obj == NULL) return;

  delete_(obj->left);
  delete_(obj->right);

  if (obj->id != -1) {
    deallocate(obj);
  }
}

/*
        deallocate obj from memeory
        if we change construct we must change this as well
*/
void slam_tree::deallocate(slam_obj* obj) {
  delete &(obj->id);
  delete &(obj->name);
  delete &(obj->T);
  delete &(obj->R);
  delete obj;
}

}  // namespace ArTracker

#endif
