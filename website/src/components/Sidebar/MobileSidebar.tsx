import {
  Sheet,
  SheetContent,
  SheetHeader,
  SheetTitle,
  SheetTrigger,
} from "@/components/ui/sheet"
import { Button } from "@/components/ui/button"
import { Menu } from "lucide-react"

export function MobileSidebar() {
  return (
    <Sheet>
      <SheetTrigger asChild>
        <Button variant="ghost" size="icon" className="md:hidden">
          <Menu />
          <span className="sr-only">Toggle menu</span>
        </Button>
      </SheetTrigger>
      <SheetContent side="left">
        <SheetHeader>
          <SheetTitle>Menu</SheetTitle>
        </SheetHeader>
        <nav className="grid gap-2 py-4">
          {/* Placeholder links - ideally these come from props or context */}
          <a href="/docs/intro" className="text-lg font-medium">
            Docs
          </a>
          <a href="/blog" className="text-lg font-medium">
            Blog
          </a>
        </nav>
      </SheetContent>
    </Sheet>
  )
}
