import "./globals.css";

export const metadata = {
  title: "vision_bot dashboard",
  description: "Live camera feed and status for the vision-guided rover",
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
