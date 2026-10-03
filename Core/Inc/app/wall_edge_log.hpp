#pragma once

// 壁切れの記録（wallEdge の WallEdge::Event）を PC へ送る（tools/DATA_FORMAT.md の表，<dir>/<file>.csv）。
// 列：side（0 左，1 右），x，boundary（対応がなければ NaN），offset（x − boundary），shift，velocity。
// 探索（app/search.cpp）と壁切れの試験（test/wall_edge_test.cpp）で共有する
namespace wall_edge_log {
// dir / file は送った後も残る文字列を渡す（dumpLast() がもう一度使う）
void dump(const char* dir, const char* file);

// 最後に dump() した名前でもう一度送る（LogDump メニュー。受信に失敗したときの取り直し）。
// まだ dump() していなければ何もしない
void dumpLast();
}
