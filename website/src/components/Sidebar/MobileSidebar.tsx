import {
  Sheet,
  SheetContent,
  SheetHeader,
  SheetTitle,
  SheetTrigger,
} from "@/components/ui/sheet"
import { Menu, BookOpen, FileText } from "lucide-react"
import { useSafeColorMode } from "../../hooks/useSafeColorMode"

export function MobileSidebar() {
  const { isDark } = useSafeColorMode()

  return (
    <Sheet>
      <SheetTrigger asChild>
        <button
          className={`md:hidden flex items-center justify-center w-10 h-10 rounded-lg transition-colors ${
            isDark
              ? 'bg-slate-800/60 hover:bg-slate-700/80 text-slate-300'
              : 'bg-slate-100 hover:bg-slate-200 text-slate-700'
          }`}
        >
          <Menu className="w-5 h-5" />
          <span className="sr-only">Toggle menu</span>
        </button>
      </SheetTrigger>
      <SheetContent
        side="left"
        className={
          isDark
            ? 'bg-slate-950 border-slate-800'
            : 'bg-white border-slate-200'
        }
      >
        <SheetHeader>
          <SheetTitle
            className={isDark ? 'text-slate-100' : 'text-slate-900'}
          >
            Menu
          </SheetTitle>
        </SheetHeader>
        <nav className="grid gap-2 py-4">
          <a
            href="/docs/intro"
            className={`flex items-center gap-3 px-4 py-3 rounded-lg text-base font-medium transition-colors ${
              isDark
                ? 'text-slate-300 hover:bg-slate-800/60 hover:text-emerald-400'
                : 'text-slate-700 hover:bg-slate-100 hover:text-emerald-600'
            }`}
          >
            <BookOpen className="w-5 h-5" />
            Docs
          </a>
          <a
            href="/blog"
            className={`flex items-center gap-3 px-4 py-3 rounded-lg text-base font-medium transition-colors ${
              isDark
                ? 'text-slate-300 hover:bg-slate-800/60 hover:text-emerald-400'
                : 'text-slate-700 hover:bg-slate-100 hover:text-emerald-600'
            }`}
          >
            <FileText className="w-5 h-5" />
            Blog
          </a>
        </nav>
      </SheetContent>
    </Sheet>
  )
}
