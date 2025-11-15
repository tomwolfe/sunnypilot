#pragma once

#include <memory>
#include <map>
#include <set>
#include <functional>
#include <vector>
#include <cassert>

#include "tools/cabana/dbc/dbcfile.h"

typedef std::set<int> SourceSet;
const SourceSet SOURCE_ALL = {-1};
const int INVALID_SOURCE = 0xff;
inline bool operator<(const std::shared_ptr<DBCFile> &l, const std::shared_ptr<DBCFile> &r) { return l.get() < r.get(); }

// Event system for the cabana application
struct SignalConnection {
  std::function<void()> handler;
};

class DBCManager {
public:
  DBCManager() {}
  ~DBCManager() {}
  bool open(const SourceSet &sources, const std::string &dbc_file_name, std::string *error = nullptr);
  bool open(const SourceSet &sources, const std::string &name, const std::string &content, std::string *error = nullptr);
  void close(const SourceSet &sources);
  void close(DBCFile *dbc_file);
  void closeAll();

  void addSignal(const MessageId &id, const cabana::Signal &sig);
  void updateSignal(const MessageId &id, const std::string &sig_name, const cabana::Signal &sig);
  void removeSignal(const MessageId &id, const std::string &sig_name);

  void updateMsg(const MessageId &id, const std::string &name, uint32_t size, const std::string &node, const std::string &comment);
  void removeMsg(const MessageId &id);

  std::string newMsgName(const MessageId &id);
  std::string newSignalName(const MessageId &id);

  const std::map<uint32_t, cabana::Msg> &getMessages(uint8_t source);
  cabana::Msg *msg(const MessageId &id);
  cabana::Msg* msg(uint8_t source, const std::string &name);

  std::vector<std::string> signalNames();
  inline int dbcCount() { return allDBCFiles().size(); }
  int nonEmptyDBCCount();

  const SourceSet sources(const DBCFile *dbc_file) const;
  DBCFile *findDBCFile(const uint8_t source);
  inline DBCFile *findDBCFile(const MessageId &id) { return findDBCFile(id.source); }
  std::set<DBCFile *> allDBCFiles();

  // Signal/slot replacements using callbacks
  typedef std::function<void(MessageId, const cabana::Signal*)> SignalAddedCallback;
  typedef std::function<void(const cabana::Signal*)> SignalRemovedCallback;
  typedef std::function<void(const cabana::Signal*)> SignalUpdatedCallback;
  typedef std::function<void(MessageId)> MsgUpdatedCallback;
  typedef std::function<void(MessageId)> MsgRemovedCallback;
  typedef std::function<void()> DBCFileChangedCallback;
  typedef std::function<void()> MaskUpdatedCallback;

  // Connection IDs
  int connectSignalAdded(const SignalAddedCallback& callback);
  int connectSignalRemoved(const SignalRemovedCallback& callback);
  int connectSignalUpdated(const SignalUpdatedCallback& callback);
  int connectMsgUpdated(const MsgUpdatedCallback& callback);
  int connectMsgRemoved(const MsgRemovedCallback& callback);
  int connectDBCFileChanged(const DBCFileChangedCallback& callback);
  int connectMaskUpdated(const MaskUpdatedCallback& callback);

  // Emit functions
  void emitSignalAdded(MessageId id, const cabana::Signal *sig);
  void emitSignalRemoved(const cabana::Signal *sig);
  void emitSignalUpdated(const cabana::Signal *sig);
  void emitMsgUpdated(MessageId id);
  void emitMsgRemoved(MessageId id);
  void emitDBCFileChanged();
  void emitMaskUpdated();

private:
  // Storage for callbacks
  std::vector<SignalAddedCallback> signalAddedCallbacks;
  std::vector<SignalRemovedCallback> signalRemovedCallbacks;
  std::vector<SignalUpdatedCallback> signalUpdatedCallbacks;
  std::vector<MsgUpdatedCallback> msgUpdatedCallbacks;
  std::vector<MsgRemovedCallback> msgRemovedCallbacks;
  std::vector<DBCFileChangedCallback> dbcFileChangedCallbacks;
  std::vector<MaskUpdatedCallback> maskUpdatedCallbacks;

private:
  std::map<int, std::shared_ptr<DBCFile>> dbc_files;
};

DBCManager *dbc();

std::string toString(const SourceSet &ss);
inline std::string msgName(const MessageId &id) {
  auto msg = dbc()->msg(id);
  return msg ? msg->name : UNNAMED;
}
